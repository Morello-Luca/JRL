# QP Derivation Notes

These notes derive the Hessian $H$ and gradient $g$ of the squeeze-force allocation QP by expanding the cost function of Eq. (cost_full) term by term, rather than presenting the final $H,g$ formulas without showing where each entry comes from. They mirror the structure used in the rewritten `QP Derivation` subsection of the paper.

## 0. Building blocks

From Section `Internal Wrench Decomposition`:

$$\lambda_{\text{grasp}}(x) = c^T x + \lambda_0, \qquad c = [c_L,\ c_R]^T$$

With the motion-adaptive reference $\lambda_{\mathrm{reference}}(t) = \lambda_{\mathrm{base}} + F_{\mathrm{demand}}(v(t))$, define the reference-shifted offset

$$\tilde\lambda_0(t) := \lambda_0 - \lambda_{\mathrm{reference}}(t)$$

so the tracking error is affine: $\lambda_{\text{grasp}}(x) - \lambda_{\mathrm{reference}}(t) = c^Tx + \tilde\lambda_0$.

The 12-D contact wrench splits per arm, $w = w_{\text{fixed}} + S_n x$, with $S_n = \mathrm{blkdiag}(S_{n,L}, S_{n,R})$. Each arm's normal force therefore drives only its own wrench:

$$w_L = w_{\text{fixed},L} + S_{n,L}F_{n,L}, \qquad w_R = w_{\text{fixed},R} + S_{n,R}F_{n,R}$$

Mapping through the arm Jacobians $\tau_i = J_i^T w_i$ gives affine, per-arm decoupled torque maps:

$$\tau_{c,L}(x) = \tau_{f,L} + b_L F_{n,L}, \qquad b_L = J_L^T S_{n,L}, \qquad \tau_{f,L} = J_L^T w_{\text{fixed},L}$$
$$\tau_{c,R}(x) = \tau_{f,R} + b_R F_{n,R}, \qquad b_R = J_R^T S_{n,R}, \qquad \tau_{f,R} = J_R^T w_{\text{fixed},R}$$

Because $\tau_{c,L}$ depends only on $F_{n,L}$ and $\tau_{c,R}$ only on $F_{n,R}$, the torque penalty will only ever touch the diagonal of $H$.

## 1. Term 1 — Squeeze-force tracking

$$\alpha\big(\lambda_{\text{grasp}}(x) - \lambda_{\mathrm{reference}}(t)\big)^2 = \alpha\big(c^Tx + \tilde\lambda_0\big)^2 = \alpha\,x^T(cc^T)x + 2\alpha\tilde\lambda_0\,c^Tx + \alpha\tilde\lambda_0^2$$

Contribution:
- Hessian: $\alpha c_L^2$ (diag, arm L), $\alpha c_R^2$ (diag, arm R), $\alpha c_Lc_R$ (off-diag)
- Gradient: $2\alpha\tilde\lambda_0 c_L$, $2\alpha\tilde\lambda_0 c_R$
- Constant $\alpha\tilde\lambda_0^2$ dropped (does not affect the minimizer)

## 2. Term 2 — Load balancing

With $d = [1,\ -1]^T$, $F_{n,L}-F_{n,R} = d^Tx$, so

$$\beta(F_{n,L}-F_{n,R})^2 = \beta\,x^T(dd^T)x = \beta\,x^T\begin{bmatrix}1&-1\\-1&1\end{bmatrix}x$$

Contribution:
- Hessian: $+\beta$ on both diagonal entries, $-\beta$ off-diagonal
- Gradient: none

## 3. Term 3 — Torque awareness

$$\gamma_i\lVert\tau_{c,i}(x)\rVert^2 = \gamma_i\lVert b_i\rVert^2 F_{n,i}^2 + 2\gamma_i(\tau_{f,i}^Tb_i)F_{n,i} + \gamma_i\lVert\tau_{f,i}\rVert^2, \quad i \in \{L,R\}$$

Contribution (each arm affects only its own entry — this is the source of the diagonal-only structure):
- Hessian: $\gamma_L\lVert b_L\rVert^2$ (diag, arm L), $\gamma_R\lVert b_R\rVert^2$ (diag, arm R), 0 off-diagonal
- Gradient: $2\gamma_L(\tau_{f,L}^Tb_L)$, $2\gamma_R(\tau_{f,R}^Tb_R)$
- Constants $\gamma_i\lVert\tau_{f,i}\rVert^2$ dropped

## 4. Assembled QP

Summing all three contributions and dropping constants gives $J(x) = \tfrac{1}{2}x^THx + g^Tx$ with

$$H = \begin{bmatrix}H_{11}&H_{12}\\H_{12}&H_{22}\end{bmatrix}, \qquad g = [g_L,\ g_R]^T$$

$$H_{11} = 2\big(\alpha c_L^2 + \beta + \gamma_L\lVert b_L\rVert^2\big)$$
$$H_{22} = 2\big(\alpha c_R^2 + \beta + \gamma_R\lVert b_R\rVert^2\big)$$
$$H_{12} = 2\big(\alpha c_Lc_R - \beta\big)$$
$$g_L = 2\big(\alpha\tilde\lambda_0 c_L + \gamma_L\,\tau_{f,L}\cdot b_L\big)$$
$$g_R = 2\big(\alpha\tilde\lambda_0 c_R + \gamma_R\,\tau_{f,R}\cdot b_R\big)$$

**Reading the structure:** the tracking term ($\alpha$) and the balance term ($\beta$) are the only sources of the off-diagonal coupling $H_{12}$ — they are what make the two arms' forces interact. The torque-awareness term ($\gamma_L,\gamma_R$) is diagonal-only, by construction, since each arm's torque cost depends solely on its own commanded force. This is exactly why torque-awareness can be added to the allocator "for free": it never changes the sparsity pattern or the problem size, only the diagonal stiffness and gradient bias. Making $\gamma_L \ne \gamma_R$ stiffens one diagonal entry relative to the other and shifts the optimal split toward the arm with more actuation headroom — this asymmetric weighting *is* the joint-torque-aware allocation mechanism, not a side effect of it.

---
*Note: the "Incorporating Joint Torque Limits into Bounds" (capacity-aware bounds) material and the Italian working notes on computation order/notation were not carried into the paper rewrite, since the paper's Constraints subsection was left untouched per your instruction. Flagging here in case you want that folded in as a separate extension later — it would replace/augment the fixed actuation floor $x_l$ with a dynamic, joint-torque-derived bound, which is a bigger change than the derivation reorganization done here.*




## Passo 5 — Procedura di calcolo, ordine delle operazioni ad ogni ciclo di controllo

1. **Leggi le forze tangenziali misurate** $F_{t,L}, F_{t,R}$ dai sensori F/T (nessuna derivata, nessuna stima di velocità/accelerazione).
2. **Calcola i bound rigidi** $x_u^L(t), x_u^R(t)$ dal Passo 1.
3. **Calcola il riferimento scalare** $x_u^{ref}(t)$ (min o media dei due bound, Passo 2).
4. **Costruisci il termine di offset** $\lambda_0 - x_u^{ref}(t) + SM_{target}$ e aggiornalo nel gradiente $g$.
5. **Costruisci $H, g$** completi (squeeze + bilanciamento + coppia, invariati nella struttura).
6. **Risolvi il QP** con i bound $x_u(t)$ appena calcolati.
7. **Ricostruisci il wrench totale** di output come nella versione precedente.

**Nota sull'ordine:** i bound (passo 2) devono essere calcolati **prima** della costruzione del gradiente (passo 4-5), perché il gradiente dipende da $x_u^{ref}(t)$. Nella pipeline originale l'ordine era invertito (QP costruito prima dei bound); va corretto.


## 0. Notazione

| Simbolo | Significato |
|---|---|
| $x_L, x_R \in \mathbb{R}$ | Forze normali di contatto ai due end-effector (variabili di ottimizzazione, negative = compressione) |
| $F_{t,L}, F_{t,R}$ | Componenti tangenziali della forza misurata dai sensori F/T ai polsi |
| $\mu$ | Coefficiente di attrito statico |
| $\delta$ | Margine di sicurezza sul modello d'attrito (costante piccola, incertezza di $\mu$) |
| $x_u^L, x_u^R$ | Limite superiore di compressione per braccio (bound rigido, da attrito) |
| $SM_{target}$ | Margine di sicurezza desiderato **sopra** il minimo fisico (parametro di design, costante) |
| $c_L, c_R$ | Coefficienti di proiezione dello squeeze (dalla proiezione $n_{squeeze}^T P_{int} S_n$, invariati rispetto alla tua formulazione originale) |
| $\alpha, \beta, \gamma_L, \gamma_R$ | Pesi della funzione di costo |
| $\tau_L(x_L), \tau_R(x_R)$ | Coppie ai giunti generate dalle forze di contatto |

