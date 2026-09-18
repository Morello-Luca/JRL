# Motion-Adaptive Internal Force Allocation for Dual-Arm Cooperative Manipulation

*(alt. title: "Task-on-Demand Grasp Force Optimization for Dual-Arm Object Transport")*

---

# Abstract
- Cooperative dual-arm transport needs enough squeeze force to prevent slip, but constant or over-conservative squeeze force wastes actuation effort and risks damaging fragile objects.
- We decompose the 12D dual-arm contact wrench into internal (squeeze) and external (task) components via a nullspace projection of the grasp matrix.
- We formulate a real-time 2-variable QP that allocates per-arm normal forces, minimizing squeeze effort and balancing the load between arms subject to friction-derived bounds.
- We couple the minimal grasp-force bound to the *motion state* of the manipulation task ("task-on-demand"), so the safety margin grows only while the object is in transit and relaxes at rest.
- We extend the same QP, at no extra decision-variable cost, with a joint-torque-aware penalty that biases squeeze-force allocation toward the arm with more actuation margin.
- We report internal force tracking error, friction-margin utilization, and squeeze effort with vs. without the motion-adaptive term, and with vs. without the joint-torque-aware penalty.

---

# I. Introduction

## I-A. Motivation
- Dual-arm cooperative transport needs sufficient normal (squeeze) force to prevent slip under load/inertial disturbance.
- Excess squeeze force is wasted actuation effort and risks damaging or destabilizing fragile/compliant objects.
- Human grip-force control scales with load/motion state rather than staying constant — one-line cite: Westling & Johansson 1984 (grip force / load force coupling with friction-dependent safety margin).

## I-B. Positioning
- Cooperative-manipulation force-distribution literature optimizes internal force statically or per-instant, without a motion-state-dependent target.
- Motion/slip-adaptive grip-force literature is mostly single-gripper and slip-*reactive* (triggered after slip is detected/predicted), not posed as a dual-arm allocation problem.
- Force-distribution literature rarely couples the allocation decision back to each arm's actuation margin (joint torque headroom); it is typically treated at the end-effector/wrench level only.

## I-C. Gap
- No existing framework couples: (a) nullspace-based internal/external wrench decomposition, (b) real-time QP allocation of squeeze force between two arms, and (c) a motion-derived force demand that is proactive (scheduled/estimated from the task's motion state) rather than reactive (slip-triggered).
- Joint-space actuation capacity (torque headroom, proximity to singularity/thermal limits) is not typically fed back into a real-time end-effector-level force allocator, despite the wrench-to-torque map being affine and cheap to exploit.

## I-D. Contributions
- A QP formulation for real-time squeeze-force allocation between two cooperating arms with friction-aware bounds.
- A motion-adaptive minimal-force term ("task-on-demand") that couples the desired grasp-force floor to the object's motion state, contrasted with a constant-safety-margin baseline.
- A joint-torque-capacity-aware extension of the same QP that biases allocation toward the arm with more actuation margin, added as a quadratic term with no increase in problem size or solver cost.
- Identification and a first mitigation (activation-blended gain switching) for a coupling failure mode between impedance stiffness and wrench gain during lateral translation.
- Validation on a dual xArm7 platform (`mc_rtc`).

---

# II. Related Work

## II-A. Internal/external force decomposition and QP-based load distribution
- Classical hybrid two-arm coordination and internal/external wrench split (Uchiyama & Dauchez).
- QP-based minimization of internal forces in cooperating manipulators (Nahon & Angeles, 1992).
- NLP/QP force-distribution schemes with normal-force constraints (Kwon & Lee; Kazerooni).
- Recent result: nonsqueezing internal load distributions from a generalized inverse of the grasp matrix are **not unique** — used here to justify the explicit weighted-objective choice rather than treating it as "the" optimal split.
- Admittance/QP dual-arm cobot frameworks regulating internal vs. external effort for safe bimanual interaction (BAZAR-style hierarchical QP).
- Recent (2025–2026) HQP variants for mobile / free-floating dual-arm systems — evidence the nullspace-projection + QP-allocation architecture is a current, accepted publication pattern.

## II-B. Friction-cone-constrained grasp/contact force optimization
- Canonical exact formulation: friction-cone feasibility as positive-definiteness / SDP (Buss, Hashimoto & Moore, 1996).
- Polyhedral/linear approximations of the friction cone for LP/QP tractability (Kerr & Roth; Cheng & Orin).
- Recent (2026) explicit SOCP-vs-QP tradeoff discussion for real-time grasp force control — supports the choice of a linear/QP friction bound over an exact SOCP cone for a real-time low-dimensional allocator.

## II-C. Motion- and slip-adaptive grip force control
- Biological grounding: grip force/load force coordination and friction-dependent safety margin (Westling & Johansson, 1984).
- Trajectory modulation as an alternative (not just complement) to grip-force modulation for slip prevention (Nature Machine Intelligence, 2025) — contrast: this paper treats grip force as the adaptive variable and trajectory as given, the inverse framing.
- Human hand-acceleration modulation alongside grip-force modulation for slip prevention (2024 study) — motivates a velocity/acceleration-*estimated* demand term over a purely schedule-driven one.
- Reactive slip control via internal-force optimization for multifingered hands, arguing uniform force increases perturb object pose vs. allocation-aware optimization (2026 arXiv) — closest conceptual neighbor to the allocation argument here.

## II-D. Joint-space capacity-aware force/torque allocation
- Redundancy resolution and torque-optimal force distribution in multi-arm/legged systems (classical: minimum-norm/weighted pseudoinverse torque allocation).
- Whole-body / multi-contact QP controllers that penalize joint torque proximity to limits as part of the cost (locomotion literature: contact-force QPs with torque or actuation-margin regularization).
- Positioning: this paper brings the same *idea* — actuation-margin-aware cost shaping — into a minimal, 2-variable dual-arm squeeze allocator, kept analytically quadratic so it costs nothing extra to solve in real time.

## II-E. Contact-state transition and hybrid position/force control
- Classical orthogonal position/force subspace decomposition (Raibert & Craig, 1981) — target formalism for the lateral-translation coupling issue (§VIII).
- Recent orthogonal interaction-frame decompositions for contact-rich manipulation (2026) — modern instance of the same idea, cited as the direction for the long-term fix.
- Smooth controller blending / gain interpolation across contact-state transitions (changing-contact manipulation frameworks) — precedent for the interim raised-cosine activation blend used here.

## II-F. Impact-aware contact establishment *(background only, not a claimed contribution)*
- Dual-arm impact-aware grasping via time-invariant reference spreading control (van Steen, Coşgun, van de Wouw, Saccon, IFAC 2023).
- QP-based reference spreading control for dual-arm manipulation with planned simultaneous impacts (van Steen et al., IEEE T-RO 2024).
- Preemptive impact reduction smoothly connected to contact impedance control (Arita et al., 2022/2023).
- Effective/reflected mass in operational-space control (Khatib) — grounds the reflected-mass virtual-mass scaling used at grasp establishment.
- Impact-aware task-space QP control (Wang, Dehio, Tanguy, Kheddar, IJRR 2023) — anchor for future work.

---

# III. System Overview

## III-A. Platform and FSM
- Dual xArm7 setup in `mc_rtc`.
- FSM: `Idle → Independent (reach) → Collaborative { Build-Grasp → Trajectory → Cooperative Motion }`.
- One system-diagram figure.

## III-B. Problem Statement
- At every control cycle, find per-arm normal (squeeze) forces `x = [F_{n,L}, F_{n,R}]^T` that:
  - track a desired internal grasp force level,
  - respect friction-derived and actuation bounds,
  - remain balanced between the two arms,
  - (extension, §VII) respect each arm's instantaneous joint-torque margin.

---

# IV. Internal Wrench Decomposition and Squeeze Isolation

## IV-A. System Wrench and Grasp Matrix
- 12D dual-arm wrench: $w = [w_L; w_R] \in \mathbb{R}^{12}$, with $w_L=[\tau_L; f_L]$, $w_R=[\tau_R; f_R]$.
- Grasp matrix $G \in \mathbb{R}^{6\times12}$; internal-wrench nullspace projector $P_{int} = I_{12} - G^\dagger G$.
- Internal wrench: $w_{int} = P_{int}\,w$.

## IV-B. Squeeze Subspace Isolation
- Raw squeeze direction $n_s = [0,0,0,0,1,0,0,0,0,0,-1,0]^T$.
- Orthonormal nullspace basis $N$ of $G$ (via SVD).
- Normalized, kinematically valid squeeze direction: $n_{squeeze} = \dfrac{NN^T n_s}{\lVert NN^T n_s\rVert}$.

## IV-C. Scalar Grasp Force
- $\lambda_{grasp} = n_{squeeze}^T w_{int} = n_{squeeze}^T P_{int}\,w$.
- Simplification using $P_{int} n_{squeeze} = n_{squeeze}$ and $P_{int}^T = P_{int}$: $\lambda_{grasp} = n_{squeeze}^T w$.
- Note for the paper: state explicitly *why* this simplification is legitimate (one line of proof, not a full derivation) — it's a clean, citable identity reviewers will want to see justified, not asserted.

## IV-D. Decomposing $w$ into Fixed and Optimized Parts
- $w = w_{fixed} + S_n x$, with $w_{fixed}$ = moments + tangential forces (measured, not optimized) and $S_n \in \mathbb{R}^{12\times2}$ mapping decision variables to world-frame normal-force wrench components via unit contact normals $\hat u_L, \hat u_R$.
- Affine grasp-force model: $\lambda_{grasp}(x) = c^T x + \lambda_0$, with $c^T = n_{squeeze}^T S_n$ and $\lambda_0 = n_{squeeze}^T w_{fixed}$.
- Interpretation: $c_L, c_R$ quantify how much a unit normal force on each arm contributes to squeeze; $\lambda_0$ is the squeeze already present from the fixed/tangential components.

---

# V. Motion-Adaptive Force Demand ("Task-on-Demand")

## V-A. Motivation
- Constant-safety-margin baseline (`F_static`) is simple but not motion-aware: over-conservative at rest, potentially insufficient at peak dynamic load.
- Design goal: minimal grasp-force floor should rise specifically while the object is being transported and relax at rest.

## V-B. Trajectory-Schedule-Driven Formulation (current / baseline)
- Quintic time-scaling: $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$, $\tau = t/T$.
- Considered index: acceleration shape $\ddot s(\tau) = 60\tau - 180\tau^2 + 120\tau^3$ → rejected because $|\ddot s(\tau)|$ has a double-lobe, discontinuous-looking profile that returns to zero at peak velocity (exactly when cumulative inertial/frictional risk is still present).
- Adopted index: velocity shape $\dot s(\tau) = 30\tau^2 - 60\tau^3 + 30\tau^4$ → single-lobe, continuous, zero only at the endpoints — explicit design choice, state the rationale in-text (not just the equation).
- $F_{demand} = k\cdot a_{demand}$, $a_{demand} = |\dot s(\tau)|$ (naming note: rename to avoid calling a velocity-shaped index "acceleration demand" in the final text).
- Limitation to state explicitly: this is feedforward from the *planned* trajectory clock ($\tau = t/T$), not from measured motion — decoupled from actual robot behavior if tracking lags or a constraint (speed bound, collision avoidance) intervenes.

## V-C. Motion-Estimated Formulation (proposed strengthening)
- Replace the trajectory-phase clock with a measured/estimated task-space velocity signal, e.g. average of both arms' commanded/measured EE speeds: $v_{obj,est} = \lVert \tfrac{1}{2}(\dot x_L + \dot x_R)\rVert$ (vector average before norm, to preserve directional-consistency information).
- $F_{demand} = k\cdot v_{obj,est}$ (single-lobe, non-negative by construction, no need for $|\cdot|$ on a sign-changing polynomial).
- Requires filtering (reuse existing α-β / low-pass stage) before use as a QP bound input, to avoid injecting solver-level velocity noise into commanded squeeze force.
- Generalizes across trajectory generators (quintic, S-curve, teleoperated, DS-based) since it consumes a measured signal, not a planner-specific clock.
- Optional hybrid: schedule term as smooth feedforward baseline + small proportional correction from planned-vs-measured velocity error — proposed as the ablation condition (c) in §X.

## V-D. Minimal Grasp Force Bound
- $F_{min} = \max\left(\dfrac{\lVert F_t\rVert}{\mu},\; F_{static} + F_{demand}\right)$ — per arm.
- Interpretation: floor is whichever is larger — the friction-required minimum, or the (static + motion-adaptive) safety margin.
- Implementation note: current code expresses the equivalent bound with the opposite sign convention (compressive forces negative, upper-bounded via `min`) — reconcile sign convention between notes and implementation before the camera-ready figure/equation pass so the paper and the code match exactly.

---

# VI. QP Formulation for Squeeze Force Optimization

## VI-A. Cost Function
- $J(x) = \underbrace{\alpha\,\lambda_{grasp}(x)^2}_{\text{minimize squeeze}} + \underbrace{\beta\,(F_{n,L}-F_{n,R})^2}_{\text{balance}}$, $\alpha,\beta > 0$.
- $\alpha$: penalizes total squeeze effort (crushing risk, actuation cost).
- $\beta$: penalizes load imbalance between arms (avoids one arm doing all the work).

## VI-B. Standard QP Form Derivation
- $\lambda_{grasp}(x)^2 = x^T cc^T x + 2\lambda_0 c^T x + \lambda_0^2$.
- Balance term via $d = [1,-1]^T$: $(F_{n,L}-F_{n,R})^2 = x^T dd^T x$.
- Dropping the constant $\alpha\lambda_0^2$: $J(x) = x^T(\alpha cc^T + \beta dd^T)x + 2\alpha\lambda_0 c^Tx$.
- Matching $J(x) = \tfrac12 x^THx + g^Tx$:
  - $H = 2\begin{bmatrix}\alpha c_L^2+\beta & \alpha c_Lc_R-\beta\\ \alpha c_Lc_R-\beta & \alpha c_R^2+\beta\end{bmatrix}$
  - $g = 2\alpha\lambda_0\begin{bmatrix}c_L\\c_R\end{bmatrix}$

## VI-C. Constraints
- Friction (per contact): $\lVert F_{t,L}\rVert \le \mu_L F_{n,L}$, $\lVert F_{t,R}\rVert \le \mu_R F_{n,R}$ — tangential forces taken from sensors (fixed, not optimized).
- Positivity / minimum-force floor: $F_{n,L} \ge F_{min}$, $F_{n,R} \ge F_{min}$ (§V-D) — prevents one arm from carrying all the squeeze while the other goes slack.
- State explicitly that the friction bound uses the *previous* cycle's measured tangential force to constrain the *next* commanded normal force (one-cycle lag) — a deliberate linear/QP simplification vs. an exact, jointly-optimized SOCP cone (cite §II-B).

## VI-D. Final Problem
- $\min_x J(x)$ s.t. $F_{n,L}\ge F_{min}$, $F_{n,R}\ge F_{min}$ (+ upper actuation bound).
- Solver: `Eigen::QLD`, 2 decision variables → report actual measured solve time (expect sub-millisecond; make this a one-line real-time-feasibility argument, not a claim without a number).

## VI-E. Sensitivity to α, β
- Small figure/table: sweep α (fixed β) and β (fixed α), report resulting $\lambda_{grasp}$ tracking and inter-arm imbalance — gives the reader intuition for tuning, and doubles as a robustness check.

---

# VII. Joint-Torque-Capacity-Aware Extension

## VII-A. Motivation
- The QP so far allocates squeeze force purely at the end-effector/wrench level; it has no notion of how "expensive" that allocation is for each arm's joints.
- End-effector wrenches map to joint torques via the Jacobian transpose ($\tau = J^Tw$); since $w$ is already affine in $x$, joint torques are affine in $x$ too — exploitable at zero extra variable cost.
- Goal: bias squeeze-force allocation toward the arm with more actuation margin (e.g., further from a singular configuration, less thermally loaded, or with more torque headroom), without adding decision variables or leaving the real-time QP structure.

## VII-B. Affine Torque Map
- Per-arm contact wrench: $w_L = w_{fixed,L} + S_{n,L}F_{n,L}$, $w_R = w_{fixed,R} + S_{n,R}F_{n,R}$.
- Corresponding joint-torque contributions: $\tau_{c,L}(x) = J_L^Tw_{fixed,L} + J_L^TS_{n,L}F_{n,L}$, $\tau_{c,R}(x) = J_R^Tw_{fixed,R} + J_R^TS_{n,R}F_{n,R}$ — affine in $x$, same structure as $\lambda_{grasp}(x)$ in §IV-D.

## VII-C. Extended Cost Function
- $J(x) = \alpha\,\lambda_{grasp}(x)^2 + \beta\,(F_{n,L}-F_{n,R})^2 + \gamma_L\lVert\tau_{c,L}(x)\rVert^2 + \gamma_R\lVert\tau_{c,R}(x)\rVert^2$.
- Each new term is quadratic in $x$ exactly like the existing two — contributes closed-form additions to the same $H,g$ pair (§VI-B), no change to problem dimension (still 2 variables) or solver.
- Term-by-term reading of the four-way trade-off: $\alpha$ (avoid crushing the object), $\beta$ (keep contact forces even), $\gamma_L,\gamma_R$ (avoid joint torque overload on each arm respectively).

## VII-D. Role and Tuning of $\gamma_L, \gamma_R$
- $\gamma_L=\gamma_R=0$ recovers the original §VI formulation exactly — the extension is strictly backward-compatible, useful for a clean ablation (with/without torque awareness) rather than a redesign.
- Small, equal, nonzero values (e.g., 0.01–0.1) protect joint torque margin without materially disrupting grasp-force tracking — report the smallest $\gamma$ that yields a measurable protective effect.
- Asymmetric values ($\gamma_L\neq\gamma_R$) let the allocator favor the arm with more capacity — e.g., setting $\gamma_L$ higher than $\gamma_R$ to shift squeeze effort onto the right arm when the left arm is known (a priori, per configuration/experiment) to be closer to a singularity, torque limit, or thermal derate. This is a manually-set, per-run configuration choice in the current implementation, not an online adjustment (§VII-E).
- Runtime-configurable (same `Params`-struct pattern as $\alpha,\beta$): supports live tuning/disabling on hardware without recompilation — worth stating as a practical/reproducibility point, not just a design nicety.

## VII-E. Implementation Status
- $\gamma_L,\gamma_R$ are implemented as fixed, manually-tuned entries in the same `Params` struct as $\alpha,\beta$ — constant gains, not driven by a live capacity signal (manipulability, torque headroom, temperature).
- State this plainly in the paper as the scope of the current contribution: a *static*-weight torque-aware allocator. Avoid implying closed-loop/adaptive capacity-awareness anywhere else in the text (abstract, intro contributions) — the claim is "the cost function is structured to allow it," not "the system currently senses and reacts to it."
- A state-driven $\gamma(t)$ version is future work only (§XII), not part of this paper's validated contribution.

---

# VIII. Contact Maintenance under Lateral Translation

## VIII-A. Failure Mode
- Observed during lateral (non-axial) translation: impedance spring force direction and commanded motion direction oppose each other at certain phases, causing loss of contact.
- Root cause (to state precisely in the paper): spring/wrench-gain terms tuned for the axial (squeeze) direction are not decoupled from the lateral motion-tracking terms — they compete rather than being orthogonal.

## VIII-B. Interim Mitigation (implemented, stopgap)
- Raised-cosine activation function used to smoothly deactivate spring stiffness and increase wrench gain on the manipulator, timed to the lateral-translation phase.
- Framing for the paper: an *engineering mitigation*, not the proposed contribution — present it as a documented, characterized stopgap (with its own small validation plot), not as a core claim.
- Report: activation window shape/duration, effect on contact-loss incidence before/after, any residual discontinuity at the blend boundaries.
- Cite as precedent: smooth controller-blending strategies across contact-state transitions in changing-contact manipulation frameworks (§II-E).

## VIII-C. Target Formulation (future work, flagged here not solved)
- Reformulate as orthogonal task-space separation: decompose the task space into complementary force-controlled and motion-controlled subspaces so spring/wrench-gain terms and lateral motion-tracking terms act on disjoint directions by construction, rather than being reconciled post hoc via gain blending.
- Cite the classical hybrid position/force orthogonal-subspace formulation (Raibert & Craig, 1981) and a recent interaction-frame instance (2026) as the target formalism.
- State explicitly in Discussion/Future Work: this is the theoretically clean fix; §VIII-B is the empirical bridge used to keep the current experiments running while it's developed.

---

# IX. Grasp Establishment *(brief, supporting only)*

- One paragraph + one figure: reflected-mass-scaled virtual mass during `Build-Grasp` softens first contact.
- Explicitly framed as adopting existing ideas (reflected/effective mass, preemptive impedance softening) — cite Khatib, Arita et al. — **not** presented as a contribution of this paper.
- State the limitation plainly: heuristic softening term, not a modeled impact-aware controller; full treatment (reference spreading / impact-aware QP) left as future work (§II-F, §XII).

---

# X. Experimental Validation

## X-A. Platform and Protocol
- Dual xArm7, `mc_rtc`.
- Pick–carry–place trajectory; force-demand conditions: (a) constant `F_static` baseline, (b) schedule-driven `F_demand` (§V-B), (c) motion-estimated `F_demand` (§V-C, if implemented in time), (d) hybrid (§V-C).
- Torque-awareness conditions: with vs. without the §VII extension, symmetric vs. asymmetric $\gamma_L,\gamma_R$.
- Secondary sweep: object mass and/or surface friction, if feasible.

## X-B. Metrics
- Internal force tracking error vs. $\lambda_{desired}$.
- Peak/RMS tangential force relative to the friction bound (slip-margin proxy).
- Total squeeze effort $\int \lVert x\rVert\,dt$ (efficiency).
- Peak/RMS joint torque per arm, and margin-to-limit, with vs. without §VII.
- Contact-loss incidence during lateral translation, with vs. without the §VIII-B activation blend.
- QP solve time distribution (should remain sub-millisecond even with the §VII extension, since problem size is unchanged — worth confirming and reporting explicitly).

## X-C. Figures
- FSM / system diagram.
- Internal force vs. trajectory phase, all demand conditions overlaid.
- Friction-margin utilization over the trajectory.
- Joint torque per arm, with vs. without torque-aware weighting, under an induced asymmetric-capacity scenario.
- Contact-loss / activation-blend before-after comparison.
- Solve-time histogram.

---

# XI. Discussion and Limitations

- Nonuniqueness of the internal/external decomposition (§II-A): the weighted objective is a design choice, not "the" optimal split — state what it privileges (low effort vs. balance vs. torque margin) and how that choice was made.
- Linearized, one-cycle-lagged friction bound vs. exact cone: identify the regime where it could fail (fast tangential-force transients).
- Schedule-driven vs. motion-estimated demand: generalizability tradeoff if the final version keeps the schedule-driven term as primary.
- Fixed vs. state-driven $\gamma_L,\gamma_R$: fixed weights are easy to defend but don't truly track live actuation capacity; flag this as the natural next step rather than overclaiming adaptivity that isn't implemented yet.
- Lateral-translation contact loss: activation blending is a documented mitigation, not a solved problem — orthogonal task-space separation remains open.
- Grasp-establishment heuristic (reflected mass) is not a full impact-aware controller.

---

# XII. Conclusion and Future Work

- Summary: QP-based dual-arm squeeze allocation + motion-adaptive demand term + joint-torque-capacity-aware extension, validated on dual xArm7.
- Future work:
  - Closed-loop, motion-estimated demand as primary (not fallback) formulation.
  - State-driven $\gamma_L,\gamma_R$ (manipulability, torque headroom, or thermal signal) replacing fixed tuned weights.
  - Orthogonal task-space separation to resolve the lateral-translation coupling failure (§VIII-C) as a principled replacement for the activation-blend stopgap.
  - Exact/SOCP friction handling if real-time budget allows.
  - Completing the impact-aware grasp establishment as a follow-up paper, engaging directly with the reference-spreading / impact-aware QP literature (§II-F).

---

# References *(working list — expand with full bibliographic entries before submission)*

**Internal force decomposition / dual-arm QP allocation**
- Uchiyama, M., Dauchez, P. — hybrid position/force scheme for two-arm coordination.
- Nahon, M., Angeles, J. (1992) — QP-based minimization of internal forces in cooperating manipulators.
- Kwon, W., Lee, B.H. — NLP/QP force-distribution scheme with normal-force constraints.
- Recent analysis on non-uniqueness of nonsqueezing load distributions in cooperative manipulation (2023).
- Admittance-QP dual-arm cobot framework regulating internal/external efforts (BAZAR, 2017–2018).
- Recent HQP variants for mobile/free-floating dual-arm manipulation (2025–2026).

**Grasp/contact force optimization under friction**
- Buss, M., Hashimoto, H., Moore, J.B. (1996) — *Dextrous Hand Grasping Force Optimization*, IEEE T-RA.
- Kerr, Roth; Cheng, Orin — polyhedral/linear friction-cone approximations.
- 2026 paper on SOCP-vs-QP tradeoff for real-time grasp force control.

**Motion-adaptive / slip-driven grip force**
- Westling, G., Johansson, R.S. (1984) — *Factors influencing the force control during precision grip*, Exp Brain Res 53:277–284.
- Nature Machine Intelligence (2025) — trajectory modulation vs. grip-force modulation for slip control.
- Human hand-acceleration modulation study for slip prevention (2024).
- Reactive slip control via internal-force optimization for multifingered hands (2026 arXiv).

**Joint-space capacity-aware allocation**
- Classical minimum-norm / weighted-pseudoinverse torque allocation for redundant/multi-arm systems.
- Whole-body / multi-contact QP controllers with torque- or actuation-margin-aware cost terms (locomotion / multi-contact force-distribution literature).

**Contact-state transition / hybrid position-force control**
- Raibert, M.H., Craig, J.J. (1981) — classical orthogonal position/force subspace hybrid control.
- Recent orthogonal interaction-frame decomposition for contact-rich manipulation (2026).
- Smooth controller-blending / gain interpolation across contact-state transitions (changing-contact manipulation frameworks).

**Impact-aware contact establishment (background only)**
- van Steen, J.J., Coşgun, A., van de Wouw, N., Saccon, A. — Dual-Arm Impact-Aware Grasping through Time-Invariant Reference Spreading Control, IFAC-PapersOnLine, 2023.
- van Steen, J.J., van den Brandt, G., van de Wouw, N., Kober, J., Saccon, A. — QP-Based Reference Spreading Control for Dual-Arm Robotic Manipulation with Planned Simultaneous Impacts, IEEE T-RO, 2024.
- Arita, H., Nakamura, H., Fujiki, T., Tahara, K. — Smoothly Connected Preemptive Impact Reduction and Contact Impedance Control, 2022/2023.
- Khatib, O. — effective/reflected mass in operational-space control.
- Wang, Y., Dehio, N., Tanguy, A., Kheddar, A. — Impact-Aware Task-Space Quadratic-Programming Control, IJRR, 2023.