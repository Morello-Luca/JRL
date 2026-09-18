# Grasp Force Distribution

objective: generate a reference for the internal loop force controller of the impedance tasks.

variable to optimize: normal contact forces

$$ x = [F_{n,L} F_{n_R}]^T $$

Motivation: optimizing just the internal forces, can create infinite distributions of $$ x = [F_{n,L} F_{n_R}]^T $$ that can lead to the same internal forces

## Constraints
for each contact:
### Friction
 - $$|| F_{t,L}|| \leq \mu_L F_{t,L}  $$ 
  - $$|| F_{t,R}|| \leq \mu_L F_{t,R}  $$  
tangential forces are already known from the sensors
### Positivity
 - $$F_{t,L} \geq F_{min}  $$ 
 - $$F_{t,R} \geq F_{min}  $$
to avoid the case of one arm pressing all forces and the other dont

## Internal Force
 - $$ P_{int} = I-G^+ G $$
 - $$ \lambda = P_{int}*[w_{L} w_{R}]^T $$
 - Squeeze direction: $$ n_s  $$ 
 - Null Space projection of squeeze: $$ n_squeeze = N * (N.transpose() * n_s); $$
 - Normalize: $$n_squeeze.stableNormalize()$$
 - $$ \lambda_{grasp} = n_squeeze.transpose() * \lambda $$


## Minimal Contact Force

### Static Contact Force
as fallback safety option

### On-Demand Force
since we use the quintic polynomal trajectory for a offline planning, we know beforehand what acceleration the object requires to move from the starting to the final point.
we can create an index of aggresivity of of the trajectory that regulate the minimal grasp

in our case:
$$ s(t) = 10 \tau^3-15\tau^4+6\tau^5  $$
with $$ \tau = t/T $$

the acceleration with respect to $$\tau$$
$$ \frac{d^2s}{d\tau^2} = 60\tau-180\tau^2+120\tau^3 $$
we can use this index $$ a_{demand} =  |60\tau-180\tau^2+120\tau^3| $$
that initially is zero, it grows during the trajectory and return zero when the trajectory is concluded.


$$ F_{demand} = k \cdot a_{demand}  $$

### friction
we take the force $$F_{Friction,L}$$ and $$F_{Friction,R}$$ that respect this constraints
$$ ||F_{t,L}|| \leq \mu_L F_{n,L}  $$
$$  ||F_{t,R}|| \leq \mu_R F_{n,R} $$



### Minimal Force 
$$F_{min} = \max \left( \frac{||F_{t}||}{\mu} , F_{static} + F_{demand}\right)$$


## Cost Function

$$ J =  \underbrace{\alpha (\lambda_{grasp})^2}_{minimum Squeeze force} +
        \underbrace{\beta (F_{n,L}-F_{n,R})^2}_{Balancing}  
         $$

## Final Problem

$$ \min_{w_{n,L},w_{n,R}} J $$
subject to:
$$ F_{n,L} \geq F_{min} $$
$$ F_{n,R} \geq F_{min} $$



# Contact Force Optimization Problem

## 1. Defining the System Wrench Vector ($w$)

We represent the total 12-dimensional wrench vector $w \in \mathbb{R}^{12}$ exerted by both end-effectors in the world frame:

$$w = \begin{bmatrix} w_L \\ w_R \end{bmatrix} \in \mathbb{R}^{12}$$

Where:
- $w_L = \begin{bmatrix} \tau_L \\ f_L \end{bmatrix} \in \mathbb{R}^6$ is the wrench of the Left arm (moments $\tau_L$, forces $f_L$).
- $w_R = \begin{bmatrix} \tau_R \\ f_R \end{bmatrix} \in \mathbb{R}^6$ is the wrench of the Right arm (moments $\tau_R$, forces $f_R$).

## Internal Wrench Projection ($P_{\text{int}}$)
Let $G \in \mathbb{R}^{6 \times 12}$ be the Grasp Matrix. The internal wrenches are the components of $w$ that lie in the null space of $G$ (i.e., forces that cancel each other out and do not cause object motion):

$$P_{\text{int}} = I_{12} - G^{\dagger} G \quad \in \mathbb{R}^{12 \times 12}$$

Where $G^{\dagger}$ is the Moore-Penrose pseudoinverse of $G$, and $I_{12}$ is the $12 \times 12$ identity matrix.
The internal wrench vector $w_{\text{int}} \in \mathbb{R}^{12}$ is:

$$w_{\text{int}} = P_{\text{int}} \, w$$

## 3. Squeeze Subspace Isolation ($n_{\text{squeeze}}$)

To isolate only the pure squeeze direction from the entire null space:

1. Let $n_s \in \mathbb{R}^{12}$ be the desired raw squeeze direction vector:

$$n_s = \begin{bmatrix} 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & -1 & 0 \end{bmatrix}^T$$

2. Let $N \in \mathbb{R}^{12 \times k}$ be an orthonormal basis matrix for $\text{null}(G)$, obtained via Singular Value Decomposition (SVD) of $G$.

3. he valid kinematic squeeze vector $n_{\text{squeeze}} \in \mathbb{R}^{12}$ is obtained by projecting $n_s$ onto $N$ and normalizing:

$$n_{\text{squeeze}} = \frac{N N^T n_s}{\Vert{}N N^T n_s\Vert{}}$$

## 4. Scalar Grasp Internal Force ($\lambda_{\text{grasp}}$)
The scalar grasp force $\lambda_{\text{grasp}} \in \mathbb{R}$ is the projection of the internal wrench vector $w_{\text{int}}$ onto the normalized squeeze direction $n_{\text{squeeze}}$:

$$\lambda_{\text{grasp}} = n_{\text{squeeze}}^T w_{\text{int}}$$
Substituting $w_{\text{int}} = P_{\text{int}} \, w$:
$$\lambda_{\text{grasp}} = n_{\text{squeeze}}^T P_{\text{int}} \, w$$
Since $n_{\text{squeeze}}$ by definition already lies entirely within the null space of $G$ (meaning $P_{\text{int}} \, n_{\text{squeeze}} = n_{\text{squeeze}}$) and $P_{\text{int}}$ is symmetric ($P_{\text{int}}^T = P_{\text{int}}$), we have:

$$n_{\text{squeeze}}^T P_{\text{int}} = (P_{\text{int}} \, n_{\text{squeeze}})^T = n_{\text{squeeze}}^T$$

Therefore, the mathematical formulation of $\lambda_{\text{grasp}}$ simplifies directly to:

$${\lambda_{\text{grasp}} = n_{\text{squeeze}}^T \, w}$$

## 5. Reformulation in Optimization Problem

### 5.1 Decomposing the System Wrench ($w$)
The total 12D wrench vector $w$ consists of the normal force components (which we want to optimize) and the tangential/moment components (which are already known/fixed from measurements or trajectories):

$$w = w_{\text{fixed}} + w_{\text{normal}}(x)$$

Where:
- $w_{\text{fixed}} \in \mathbb{R}^{12}$ contains the moments ($\tau_L, \tau_R$) and tangential forces ($F_{t,L}, F_{t,R}$) which do not depend on $x$.
- $w_{\text{normal}}(x) \in \mathbb{R}^{12}$ maps the decision variables $x = \begin{bmatrix} F_{n,L} \\ F_{n,R} \end{bmatrix}$ into the full wrench space.

### 5.2 Mapping Normal Forces to 12D Wrench Space
Let $\hat{u}_L, \hat{u}_R \in \mathbb{R}^3$ be the unit direction vectors for the normal force components of the Left and Right arms in the world frame (e.g., pointing along the contact normal axes).We can construct a Selection Matrix $S_n \in \mathbb{R}^{12 \times 2}$ that maps $x$ directly to the 12D wrench vector:

$$S_n = \begin{bmatrix}  0_{3 \times 1} & 0_{3 \times 1} \\  \hat{u}_L & 0_{3 \times 1} \\  0_{3 \times 1} & 0_{3 \times 1} \\  0_{3 \times 1} & \hat{u}_R  \end{bmatrix}_{12 \times 2}$$

Thus, the normal component of the wrench vector is expressed as:

$$w_{\text{normal}}(x) = S_n x = S_n \begin{bmatrix} F_{n,L} \\ F_{n,R} \end{bmatrix}$$

### 5.3 Expressing $\lambda_{\text{grasp}}$ as a Function of $x$

Using the simplified definition of $\lambda_{\text{grasp}}$ from Section 4:

$$\lambda_{\text{grasp}}(x) = n_{\text{squeeze}}^T w$$

Substituting $w = w_{\text{fixed}} + S_n x$:

$$\lambda_{\text{grasp}}(x) = n_{\text{squeeze}}^T \left( w_{\text{fixed}} + S_n x \right)$$

Distributing the inner product:

$$\lambda_{\text{grasp}}(x) = \left( n_{\text{squeeze}}^T S_n \right) x + \left( n_{\text{squeeze}}^T w_{\text{fixed}} \right)$$

### 5.4 Compact Linear Vector Representation
We define the linear mapping vector $c \in \mathbb{R}^2$ and the scalar offset $\lambda_0 \in \mathbb{R}$:

1. Mapping Vector $c \in \mathbb{R}^2$:

$$c^T = n_{\text{squeeze}}^T S_n = \begin{bmatrix} c_L & c_R \end{bmatrix}$$

$$c^T = n_{\text{squeeze}}^T S_n = \begin{bmatrix} c_L & c_R \end{bmatrix}$$Where $c_L$ and $c_R$ represent how much a unit normal force on the Left and Right arm contributes to the squeeze direction.

2. Baseline Offset $\lambda_0 \in \mathbb{R}$:

$$\lambda_0 = n_{\text{squeeze}}^T w_{\text{fixed}}$$

This captures any existing internal squeeze force contributed by the fixed/tangential forces.

This gives us the final affine (linear) expression for $\lambda_{\text{grasp}}$ as a function of $x$:

$${\lambda_{\text{grasp}}(x) = c^T x + \lambda_0 = c_L F_{n,L} + c_R F_{n,R} + \lambda_0}$$

# Cost Function & Matrix Derivation

## 1. General Definition of the Cost Function
Our goal is to optimize the normal contact forces $x = \begin{bmatrix} F_{n,L} \\ F_{n,R} \end{bmatrix}$ using two objectives:
1. Minimize the Grasp Internal Force ($\lambda_{\text{grasp}}$): Keep the squeeze force as small as possible to avoid crushing the object.
2. Balance Normal Forces ($F_{n,L} - F_{n,R}$): Distribute the normal force evenly between both arms to prevent one arm from doing all the work.

We define the primary cost function $J(x)$ as a weighted sum of squares:

$$ J =  \underbrace{\alpha (\lambda_{grasp})^2}_{minimum Squeeze force} +
        \underbrace{\beta (F_{n,L}-F_{n,R})^2}_{Balancing}  
         $$
  
Where $\alpha > 0$ and $\beta > 0$ are scalar weighting factors.

## 2. Expressing $J(x)$ in Vector Form
Using our decision variable $x = \begin{bmatrix} F_{n,L} \\ F_{n,R} \end{bmatrix}$:

### 2.1 First Term: Squeeze Force Objective
From our previous step, $\lambda_{\text{grasp}}(x) = c^T x + \lambda_0$. Squaring this term gives:

$$\left(\lambda_{\text{grasp}}(x)\right)^2 = (c^T x + \lambda_0)^2 = (c^T x + \lambda_0)(c^T x + \lambda_0)$$

Expanding:

$$\left(\lambda_{\text{grasp}}(x)\right)^2 = x^T c c^T x + 2 \lambda_0 c^T x + \lambda_0^2$$

Multiplying by $\alpha$:

$$J_1(x) = \alpha x^T (c c^T) x + 2 \alpha \lambda_0 c^T x + \alpha \lambda_0^2$$

### 2.2 Second Term: Force Balancing Objective
The difference $(F_{n,L} - F_{n,R})$ can be written using a constant difference vector $d = \begin{bmatrix} 1 \\ -1 \end{bmatrix}$:

$$F_{n,L} - F_{n,R} = d^T x$$

Squaring this term gives:
$$(F_{n,L} - F_{n,R})^2 = (d^T x)^2 = x^T (d d^T) x$$
Where the outer product matrix $d d^T \in \mathbb{R}^{2 \times 2}$ is:

$$d d^T = \begin{bmatrix} 1 \\ -1 \end{bmatrix} \begin{bmatrix} 1 & -1 \end{bmatrix} = \begin{bmatrix} 1 & -1 \\ -1 & 1 \end{bmatrix}$$

Multiplying by $\beta$:

$$J_2(x) = \beta x^T (d d^T) x = \beta x^T \begin{bmatrix} 1 & -1 \\ -1 & 1 \end{bmatrix} x$$

### 2.3 Total Expanded Cost Function
Combining $J_1(x)$ and $J_2(x)$ (ignoring the constant scalar term $\alpha \lambda_0^2$, as it does not affect the optimal $x$):

$$J(x) = x^T \left( \alpha c c^T + \beta d d^T \right) x + \left( 2 \alpha \lambda_0 c^T \right) x$$

## 3. Extracting the Standard Quadratic Form
In standard Quadratic Programming (QP), a cost function is written as:

$$J(x) = \frac{1}{2} x^T H x + g^T x$$

Comparing $J(x)$ directly to the standard QP form:

### 3.1 Hessian Matrix ($H \in \mathbb{R}^{2 \times 2}$)

The matrix $H$ is the second derivative (Hessian) of $J(x)$ with respect to $x$:

$$\frac{1}{2} H = \alpha c c^T + \beta d d^T \implies \mathbf{H = 2 \left( \alpha c c^T + \beta d d^T \right)}$$

Expanding $H$ in terms of scalar components $c = \begin{bmatrix} c_L \\ c_R \end{bmatrix}$:

$$c c^T = \begin{bmatrix} c_L^2 & c_L c_R \\ c_L c_R & c_R^2 \end{bmatrix}$$

Substituting $c c^T$ and $d d^T$:

$$\mathbf{H = 2 \begin{bmatrix} \alpha c_L^2 + \beta & \alpha c_L c_R - \beta \\ \alpha c_L c_R - \beta & \alpha c_R^2 + \beta \end{bmatrix}}$$

### 3.2 Gradient Vector ($g \in \mathbb{R}^2$)
The linear term $g$ is derived directly from the linear component of $J(x)$:

$$g^T x = 2 \alpha \lambda_0 c^T x \implies \mathbf{g^T = 2 \alpha \lambda_0 c^T}$$

Taking the transpose to get $g$ as a column vector:

$$\mathbf{g = 2 \alpha \lambda_0 c = 2 \alpha \lambda_0 \begin{bmatrix} c_L \\ c_R \end{bmatrix}}$$

## 4. Summary of Cost Function Matrices

$$
{\begin{aligned} \mathbf{H} &= 2 \begin{bmatrix} \alpha c_L^2 + \beta & \alpha c_L c_R - \beta \\ \alpha c_L c_R - \beta & \alpha c_R^2 + \beta \end{bmatrix} \\ 
\mathbf{g} &= 2 \alpha \lambda_0 \begin{bmatrix} c_L \\ c_R \end{bmatrix} \end{aligned}}
$$

# Joint-Space Capacity-Aware Force Allocation

To elevate the standard end-effector force allocation to be joint-space capacity-aware, we extend the QP to directly penalize the joint torques required to generate the contact forces. This ensures the optimizer favors force distributions that keep the joint torques small (i.e. protecting weak actuators or joints near singular configurations).

## 1. Mapping Contact Wrenches to Joint Torques
The relation between end-effector contact wrenches $w_L, w_R \in \mathbb{R}^6$ (forces and moments) and the joint torques $\tau_L \in \mathbb{R}^{n_L}, \tau_R \in \mathbb{R}^{n_R}$ (where $n_L, n_R$ are the number of joints of each arm, e.g., 7 for xArm7) is given by the arm Jacobians $J_L \in \mathbb{R}^{6 \times n_L}$ and $J_R \in \mathbb{R}^{6 \times n_R}$:

$$\tau_L = J_L^T w_L$$
$$\tau_R = J_R^T w_R$$

## 2. Expressing Joint Torques as Affine Functions of $x$
Recall that the total 12D wrench vector $w = \begin{bmatrix} w_L \\ w_R \end{bmatrix}$ is decomposed into:

$$w = w_{\text{fixed}} + S_n x$$

We can segment $w_{\text{fixed}}$ and $S_n$ into left and right arm blocks:

$$w_{\text{fixed}} = \begin{bmatrix} w_{\text{fixed},L} \\ w_{\text{fixed},R} \end{bmatrix}, \quad S_n = \begin{bmatrix} S_{n,L} & 0_{6 \times 1} \\ 0_{6 \times 1} & S_{n,R} \end{bmatrix}$$

Where:
- $w_{\text{fixed},i} \in \mathbb{R}^6$ is the measured/fixed wrench (moments and tangential forces) on arm $i$.
- $S_{n,i} = \begin{bmatrix} 0_{3 \times 1} \\ \hat{u}_i \end{bmatrix} \in \mathbb{R}^{6 \times 1}$ maps the scalar normal force $F_{n,i}$ to its 6D wrench on arm $i$.

Thus:
$$w_L = w_{\text{fixed},L} + S_{n,L} F_{n,L}$$
$$w_R = w_{\text{fixed},R} + S_{n,R} F_{n,R}$$

Substituting these back into the torque mappings, we obtain the joint torques as affine functions of the decision variables $F_{n,L}$ and $F_{n,R}$:

$$\tau_L(F_{n,L}) = \tau_{fixed,L} + b_L F_{n,L}$$
$$\tau_R(F_{n,R}) = \tau_{fixed,R} + b_R F_{n,R}$$

Where:
- $\tau_{fixed,i} = J_i^T w_{\text{fixed},i} \in \mathbb{R}^{n_i}$ is the joint torque vector from the fixed end-effector components (gravity, payload, friction).
- $b_i = J_i^T S_{n,i} \in \mathbb{R}^{n_i}$ is the torque contribution vector per unit normal force.

## 3. Formulating the Joint Torque Minimization Penalty
We add a weighted joint torque norm term to the primary cost function:

$$J_{torque}(x) = \gamma_L \|\tau_L(F_{n,L})\|^2 + \gamma_R \|\tau_R(F_{n,R})\|^2$$

Where $\gamma_L, \gamma_R \geq 0$ are the torque optimization weights. Expanding each term:

$$\|\tau_i(F_{n,i})\|^2 = (\tau_{fixed,i} + b_i F_{n,i})^T (\tau_{fixed,i} + b_i F_{n,i}) = (b_i^T b_i) F_{n,i}^2 + (2 \tau_{fixed,i}^T b_i) F_{n,i} + \tau_{fixed,i}^T \tau_{fixed,i}$$

Using vector form $x = \begin{bmatrix} F_{n,L} \\ F_{n,R} \end{bmatrix}$, the total torque cost (ignoring the constant $\tau_{fixed,i}^T \tau_{fixed,i}$ term) is:

$$J_{torque}(x) = \frac{1}{2} x^T H_{\tau} x + g_{\tau}^T x$$

Where the Hessian contribution $H_{\tau} \in \mathbb{R}^{2 \times 2}$ is diagonal:

$$H_{\tau} = 2 \begin{bmatrix} \gamma_L (b_L^T b_L) & 0 \\ 0 & \gamma_R (b_R^T b_R) \end{bmatrix}$$

And the gradient contribution $g_{\tau} \in \mathbb{R}^2$ is:

$$g_{\tau} = 2 \begin{bmatrix} \gamma_L (\tau_{fixed,L}^T b_L) \\ \gamma_R (\tau_{fixed,R}^T b_R) \end{bmatrix}$$

## 4. Incorporating Joint Torque Limits into Bounds (Capacity-Aware Bounds)
To enforce physical joint torque limits $\tau_{min,i} \leq \tau_i \leq \tau_{max,i}$ directly in the 2D QP, we project the joint torque inequalities onto the decision variables.

For each joint $j \in \{1, \dots, n_i\}$ of arm $i \in \{L, R\}$:

$$\tau_{min,i,j} \leq \tau_{fixed,i,j} + b_{i,j} F_{n,i} \leq \tau_{max,i,j}$$

This yields linear bounds on $F_{n,i}$ depending on the sign of the joint sensitivity coefficient $b_{i,j}$:

1. If $b_{i,j} > 0$:
   $$\frac{\tau_{min,i,j} - \tau_{fixed,i,j}}{b_{i,j}} \leq F_{n,i} \leq \frac{\tau_{max,i,j} - \tau_{fixed,i,j}}{b_{i,j}}$$

2. If $b_{i,j} < 0$:
   $$\frac{\tau_{max,i,j} - \tau_{fixed,i,j}}{b_{i,j}} \leq F_{n,i} \leq \frac{\tau_{min,i,j} - \tau_{fixed,i,j}}{b_{i,j}}$$

By checking these conditions for all joints across both arms, we compute dynamic joint-capacity bounds $[F_{joint\_min,i}, F_{joint\_max,i}]$ and intersect them with the friction/positivity bounds $xl$ and $xu$:

$$xl_i = \max(xl_i, F_{joint\_min,i})$$
$$xu_i = \min(xu_i, F_{joint\_max,i})$$

This formulation keeps the QP extremely fast (2 variables), guarantees that joint-space torque limits are respected in real time, and optimizes joint-space energy efficiency.



# 26-06
# Controllo di Forza Cooperativo Dual-Arm — Formulazione Finale con Margine di Sicurezza Dinamico

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

---

## Passo 1 — Vincolo fisico rigido (limite di Coulomb)

Il minimo di compressione necessario per non far scivolare l'oggetto **resta un vincolo rigido**, non negoziabile, calcolato dalla forza tangenziale *misurata* (nessuna derivata numerica, nessun rumore aggiunto):

$$
x_u^L = -\frac{|F_{t,L}|}{\mu} - \delta, \qquad x_u^R = -\frac{|F_{t,R}|}{\mu} - \delta
$$

Vincoli box del QP:
$$
x_l \le x_L \le x_u^L, \qquad x_l \le x_R \le x_u^R
$$

**Perché è rigido e non nel costo:** è una condizione di sicurezza assoluta (non-scivolamento). Non deve mai essere violata, quindi va nei bound del QP, non in un termine soft che potrebbe essere "battuto" da un altro termine.

---

## Passo 2 — Margine di sicurezza dinamico (il concetto chiave)

**Errore della versione precedente:** il termine di squeeze puntava a $0$ (forza interna nulla). Poiché il vincolo del Passo 1 impone comunque una compressione minima, l'ottimizzatore finiva sempre schiacciato sul bound — rendendo $\gamma_L,\gamma_R$ ininfluenti (paradosso descritto nell'analisi iniziale).

**Correzione:** il termine di squeeze non deve puntare al bound stesso ($x_u$), ma a un punto **oltre** il bound, a distanza $SM_{target}$:

$$
x_{ref} = x_u - SM_{target}
$$

> ⚠️ **Nota sul segno.** $x_u$ è negativo (compressione). "Aumentare il margine di sicurezza" significa comprimere *di più*, cioè andare *più negativo*. Quindi $SM_{target} > 0$ va **sottratto** a $x_u$ per ottenere un riferimento più negativo di $x_u$. Verificare numericamente prima del deploy: es. $x_u = -10\,N$, $SM_{target}=5\,N$ → $x_{ref} = -15\,N$ (più compressione, corretto).

Poiché il tuo QP ha un unico scalare $\lambda_0$ (non un riferimento per braccio separato), serve un solo valore di riferimento a partire da $x_u^L, x_u^R$. Due opzioni:

- **Conservativa (consigliata):** $x_u^{ref} = \min(x_u^L, x_u^R)$ — il margine insegue sempre il braccio messo peggio.
- **Media:** $x_u^{ref} = \frac{1}{2}(x_u^L + x_u^R)$ — più semplice, ma può sottostimare il bisogno del braccio più caricato.

Questo valore **non sostituisce** $c_L, c_R$ (che restano quelli calcolati dalla proiezione fisica e determinano *come* si distribuisce lo sforzo tra i due bracci) — serve solo a definire *quanto* squeeze totale è il target di riferimento.

---

## Passo 3 — Funzione di costo finale

$$
f(x_L, x_R) = \alpha\big(c_L x_L + c_R x_R + \lambda_0 - x_u^{ref}(t) + SM_{target}\big)^2 \;+\; \beta(x_L - x_R)^2 \;+\; \gamma_L\|\tau_L(x_L)\|^2 \;+\; \gamma_R\|\tau_R(x_R)\|^2
$$

soggetta a:
$$
x_l \le x_L \le x_u^L(t), \qquad x_l \le x_R \le x_u^R(t)
$$

### Interpretazione termine per termine

1. **Termine di squeeze ($\alpha$):** non spinge più verso zero forza interna, ma verso un punto **a distanza $SM_{target}$ oltre il minimo fisico corrente**. È un target che *si muove* con il carico reale (via $x_u^{ref}(t)$, aggiornato ogni ciclo da $F_t$ misurato), invece di essere una costante fissa o zero.
2. **Termine di bilanciamento ($\beta$):** invariato — penalizza differenza di forza tra i due bracci.
3. **Termine di coppia ($\gamma_L,\gamma_R$):** ora ha davvero un ruolo, perché il vincolo non è più sempre attivo esattamente dove punta anche il termine $\alpha$: il QP ha un margine reale ($SM_{target}$) dentro cui $\gamma$ può spostare la soluzione per scaricare il braccio più stressato.

### Perché "squeeze maggiore dei bound" e non uguale

Il punto centrale della riformulazione: **il target di costo deve stare strettamente oltre il bound, mai esattamente su di esso.** Se il target coincidesse col bound (come accadeva quando puntava a $x_u$ direttamente, o a $0$ con bound sempre attivo), l'ottimizzatore non avrebbe mai un vero grado di libertà: la soluzione sarebbe sempre "sul muro", e qualunque peso ($\gamma$ in particolare) diventerebbe decorativo, esattamente come nel problema originale. Tenendo il target a distanza $SM_{target}>0$ dal bound, si crea uno spazio di manovra reale dentro cui i pesi $\beta, \gamma$ possono effettivamente competere e produrre soluzioni diverse.

---

## Passo 4 — Problema QP in forma canonica

Variabili: $x = [x_L, x_R]^T$

**Hessiana (2×2), invariata nella struttura rispetto alla versione precedente:**
$$
H = 2\begin{bmatrix} \alpha c_L^2 + \beta & \alpha c_Lc_R - \beta \\ \alpha c_Lc_R - \beta & \alpha c_R^2 + \beta \end{bmatrix} + H_\tau
$$
dove $H_\tau$ è il contributo dei termini $\gamma_L\|\tau_L\|^2, \gamma_R\|\tau_R\|^2$ (invariato, dipende solo da $J_L^TS_{n,L}$, $J_R^TS_{n,R}$).

**Gradiente (2×1):**
$$
g = 2\alpha\big(\lambda_0 - x_u^{ref}(t) + SM_{target}\big)\, c + g_\tau
$$
dove $c = [c_L, c_R]^T$ e $g_\tau$ è invariato rispetto alla versione precedente.

**Vincoli box, ora time-varying:**
$$
x_l \le x \le x_u(t), \qquad x_u(t) = \big[x_u^L(t),\, x_u^R(t)\big]^T
$$

---

## Passo 5 — Procedura di calcolo, ordine delle operazioni ad ogni ciclo di controllo

1. **Leggi le forze tangenziali misurate** $F_{t,L}, F_{t,R}$ dai sensori F/T (nessuna derivata, nessuna stima di velocità/accelerazione).
2. **Calcola i bound rigidi** $x_u^L(t), x_u^R(t)$ dal Passo 1.
3. **Calcola il riferimento scalare** $x_u^{ref}(t)$ (min o media dei due bound, Passo 2).
4. **Costruisci il termine di offset** $\lambda_0 - x_u^{ref}(t) + SM_{target}$ e aggiornalo nel gradiente $g$.
5. **Costruisci $H, g$** completi (squeeze + bilanciamento + coppia, invariati nella struttura).
6. **Risolvi il QP** con i bound $x_u(t)$ appena calcolati.
7. **Ricostruisci il wrench totale** di output come nella versione precedente.

**Nota sull'ordine:** i bound (passo 2) devono essere calcolati **prima** della costruzione del gradiente (passo 4-5), perché il gradiente dipende da $x_u^{ref}(t)$. Nella pipeline originale l'ordine era invertito (QP costruito prima dei bound); va corretto.

---

## Punti da verificare prima del deploy (checklist)

# Controllo di Forza Cooperativo Dual-Arm — Formulazione Finale con Margine di Sicurezza Dinamico

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

---

## Passo 1 — Vincolo fisico rigido (limite di Coulomb)

Il minimo di compressione necessario per non far scivolare l'oggetto **resta un vincolo rigido**, non negoziabile, calcolato dalla forza tangenziale *misurata* (nessuna derivata numerica, nessun rumore aggiunto):

$$
x_u^L = -\frac{|F_{t,L}|}{\mu} - \delta, \qquad x_u^R = -\frac{|F_{t,R}|}{\mu} - \delta
$$

Vincoli box del QP:
$$
x_l \le x_L \le x_u^L, \qquad x_l \le x_R \le x_u^R
$$

**Perché è rigido e non nel costo:** è una condizione di sicurezza assoluta (non-scivolamento). Non deve mai essere violata, quindi va nei bound del QP, non in un termine soft che potrebbe essere "battuto" da un altro termine.

---

## Passo 2 — Margine di sicurezza dinamico (il concetto chiave)

**Errore della versione precedente:** il termine di squeeze puntava a $0$ (forza interna nulla). Poiché il vincolo del Passo 1 impone comunque una compressione minima, l'ottimizzatore finiva sempre schiacciato sul bound — rendendo $\gamma_L,\gamma_R$ ininfluenti (paradosso descritto nell'analisi iniziale).

**Correzione:** il termine di squeeze non deve puntare al bound stesso ($x_u$), ma a un punto **oltre** il bound, a distanza $SM_{target}$:

$$
x_{ref} = x_u - SM_{target}
$$

> ⚠️ **Nota sul segno.** $x_u$ è negativo (compressione). "Aumentare il margine di sicurezza" significa comprimere *di più*, cioè andare *più negativo*. Quindi $SM_{target} > 0$ va **sottratto** a $x_u$ per ottenere un riferimento più negativo di $x_u$. Verificare numericamente prima del deploy: es. $x_u = -10\,N$, $SM_{target}=5\,N$ → $x_{ref} = -15\,N$ (più compressione, corretto).

Poiché il tuo QP ha un unico scalare $\lambda_0$ (non un riferimento per braccio separato), serve un solo valore di riferimento a partire da $x_u^L, x_u^R$. Due opzioni:

- **Conservativa (consigliata):** $x_u^{ref} = \min(x_u^L, x_u^R)$ — il margine insegue sempre il braccio messo peggio.
- **Media:** $x_u^{ref} = \frac{1}{2}(x_u^L + x_u^R)$ — più semplice, ma può sottostimare il bisogno del braccio più caricato.

Questo valore **non sostituisce** $c_L, c_R$ (che restano quelli calcolati dalla proiezione fisica e determinano *come* si distribuisce lo sforzo tra i due bracci) — serve solo a definire *quanto* squeeze totale è il target di riferimento.

---

## Passo 3 — Funzione di costo finale

$$
f(x_L, x_R) = \alpha\big(c_L x_L + c_R x_R + \lambda_0 - x_u^{ref}(t) + SM_{target}\big)^2 \;+\; \beta(x_L - x_R)^2 \;+\; \gamma_L\|\tau_L(x_L)\|^2 \;+\; \gamma_R\|\tau_R(x_R)\|^2
$$

soggetta a:
$$
x_l \le x_L \le x_u^L(t), \qquad x_l \le x_R \le x_u^R(t)
$$

### Interpretazione termine per termine

1. **Termine di squeeze ($\alpha$):** non spinge più verso zero forza interna, ma verso un punto **a distanza $SM_{target}$ oltre il minimo fisico corrente**. È un target che *si muove* con il carico reale (via $x_u^{ref}(t)$, aggiornato ogni ciclo da $F_t$ misurato), invece di essere una costante fissa o zero.
2. **Termine di bilanciamento ($\beta$):** invariato — penalizza differenza di forza tra i due bracci.
3. **Termine di coppia ($\gamma_L,\gamma_R$):** ora ha davvero un ruolo, perché il vincolo non è più sempre attivo esattamente dove punta anche il termine $\alpha$: il QP ha un margine reale ($SM_{target}$) dentro cui $\gamma$ può spostare la soluzione per scaricare il braccio più stressato.

### Perché "squeeze maggiore dei bound" e non uguale

Il punto centrale della riformulazione: **il target di costo deve stare strettamente oltre il bound, mai esattamente su di esso.** Se il target coincidesse col bound (come accadeva quando puntava a $x_u$ direttamente, o a $0$ con bound sempre attivo), l'ottimizzatore non avrebbe mai un vero grado di libertà: la soluzione sarebbe sempre "sul muro", e qualunque peso ($\gamma$ in particolare) diventerebbe decorativo, esattamente come nel problema originale. Tenendo il target a distanza $SM_{target}>0$ dal bound, si crea uno spazio di manovra reale dentro cui i pesi $\beta, \gamma$ possono effettivamente competere e produrre soluzioni diverse.

---

## Passo 4 — Problema QP in forma canonica

Variabili: $x = [x_L, x_R]^T$

**Hessiana (2×2), invariata nella struttura rispetto alla versione precedente:**
$$
H = 2\begin{bmatrix} \alpha c_L^2 + \beta & \alpha c_Lc_R - \beta \\ \alpha c_Lc_R - \beta & \alpha c_R^2 + \beta \end{bmatrix} + H_\tau
$$
dove $H_\tau$ è il contributo dei termini $\gamma_L\|\tau_L\|^2, \gamma_R\|\tau_R\|^2$ (invariato, dipende solo da $J_L^TS_{n,L}$, $J_R^TS_{n,R}$).

**Gradiente (2×1):**
$$
g = 2\alpha\big(\lambda_0 - x_u^{ref}(t) + SM_{target}\big)\, c + g_\tau
$$
dove $c = [c_L, c_R]^T$ e $g_\tau$ è invariato rispetto alla versione precedente.

**Vincoli box, ora time-varying:**
$$
x_l \le x \le x_u(t), \qquad x_u(t) = \big[x_u^L(t),\, x_u^R(t)\big]^T
$$

---

## Passo 5 — Procedura di calcolo, ordine delle operazioni ad ogni ciclo di controllo

1. **Leggi le forze tangenziali misurate** $F_{t,L}, F_{t,R}$ dai sensori F/T (nessuna derivata, nessuna stima di velocità/accelerazione).
2. **Calcola i bound rigidi** $x_u^L(t), x_u^R(t)$ dal Passo 1.
3. **Calcola il riferimento scalare** $x_u^{ref}(t)$ (min o media dei due bound, Passo 2).
4. **Costruisci il termine di offset** $\lambda_0 - x_u^{ref}(t) + SM_{target}$ e aggiornalo nel gradiente $g$.
5. **Costruisci $H, g$** completi (squeeze + bilanciamento + coppia, invariati nella struttura).
6. **Risolvi il QP** con i bound $x_u(t)$ appena calcolati.
7. **Ricostruisci il wrench totale** di output come nella versione precedente.

**Nota sull'ordine:** i bound (passo 2) devono essere calcolati **prima** della costruzione del gradiente (passo 4-5), perché il gradiente dipende da $x_u^{ref}(t)$. Nella pipeline originale l'ordine era invertito (QP costruito prima dei bound); va corretto.

---

## Punti da verificare prima del deploy (checklist)

- [ ] **Segno di $SM_{target}$** nell'offset — verificare con un caso numerico concreto che il target finale sia effettivamente *più* compressivo del bound, non meno.
- [ ] **Scelta min vs media** per $x_u^{ref}(t)$ — la versione min è più conservativa e coerente con un vincolo di sicurezza per-braccio; documentarla esplicitamente nel paper come scelta di design.
- [ ] **Taratura di $SM_{target}$** — deve essere abbastanza piccolo da non sprecare inutilmente coppia, abbastanza grande da lasciare un margine di manovra reale a $\beta, \gamma$. Va giustificato con un test di sensibilità, non lasciato come numero arbitrario.
- [ ] **$\delta$ (margine sul modello d'attrito) e $SM_{target}$ (margine di costo) sono due cose distinte** — non vanno sommati o confusi: $\delta$ sta nel vincolo rigido (sicurezza assoluta), $SM_{target}$ sta nel costo (obiettivo morbido).

(A−xref​)2=A2−2Axref​+xref2​