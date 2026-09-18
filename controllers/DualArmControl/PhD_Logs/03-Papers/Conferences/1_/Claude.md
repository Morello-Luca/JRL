# Motion-Adaptive Internal Force Optimization for Dual-Arm Cooperative Manipulation
### Review, Positioning, and Paper Outline

---

## 1. What you actually have (reading the code)

Four pieces:

- **`DualArmControl.cpp`** — an `mc_rtc` FSM controller for two xArm7 arms: `IDLE → INDEPENDENT (reach) → COLLABORATIVE {BUILD_GRASP → TRAJECTORY → COOPERATIVE_MOTION}`. During `BUILD_GRASP` it shrinks the impedance virtual mass using a **reflected-mass-scaled** term (`M_virtual_z = r * min(mLeft, mRight)`) to soften contact — this is your "impact-aware" piece.
- **`DualArmControlForce.cpp`** — internal force estimation via nullspace projection (`Pint = I - Gpinv*G`), a squeeze-direction estimator (`n_squeeze`), three tunable α-β filters + five low-pass variants for the measured internal force, and `DemandForces(K)`, a **feedforward force-demand profile shaped by the normalized trajectory phase `t_norm`** (the derivative of a quintic smoothstep — i.e., a bell-shaped "velocity-like" envelope that peaks mid-trajectory and vanishes at the endpoints).
- **`DualArmQPOptimizer.{h,cpp}`** — a 2-variable QP (solved with `Eigen::QLD`) that finds the optimal normal (squeeze) forces at the two contacts, minimizing a weighted combination of (a) the resulting internal force error relative to `lambda0` and (b) an inter-arm balance term, subject to friction-derived bounds.

**This is the real contribution:** you're coupling *(i)* a nullspace-based internal/external wrench decomposition, *(ii)* a small, real-time QP that allocates squeeze force between two arms with a friction-aware bound, and *(iii)* a **trajectory-phase feedforward term that raises the grasp force specifically while the object is moving**, not as a constant safety margin. That specific combination — motion-triggered demand + QP-based dual-arm allocation — is not something I found as a single unit in the literature (see §3). It's your most publishable asset.

The reflected-mass contact-establishment logic is a reasonable heuristic but is not yet a method (see §2.3) — treat it as supporting infrastructure, not a claim.

### 1.1 Bugs / inconsistencies to fix before anyone reviews this

These aren't nitpicks — a reviewer (or a compiler) will hit them immediately:

1. **`DualArmQPOptimizer.h` doesn't compile as shown**: `buildQPProblem(...)` is missing a trailing `;`, so it's parsed as continuing into `computeBounds`.
2. **Constructor mismatch**: header declares `DualArmQPOptimizer()` (no args); `.cpp` defines `DualArmQPOptimizer(const Params&)`. Pick one — given `Params` are also passed into `optimize()` every call, I'd drop the constructor argument entirely (see #4).
3. **`computeBounds` signature mismatch**: header has `(fL, fR, F_demand)`; `.cpp` implements `(fL, fR, F_demand, const Params&)`; the call site in `optimize()` passes only 3 args. Won't link.
4. **Redundant/conflicting `Params` sources**: `buildQPProblem` takes `params` as an argument but its body reads the member `params_` instead (set once at construction). Meanwhile `optimize()` receives a fresh `params` argument every call from `DualArmControlForce.cpp`, which is never used to update `params_`. So today, `alpha`/`beta` are effectively frozen at construction while `mu`/`F_static` are re-read live in `computeBounds`. Decide: either `Params` is per-call (drop `params_` member, thread the argument through consistently — simplest, and matches the fact that `qpOptimizer_.optimize()` is called with a freshly-built `qpParams` every control cycle in `DualArmControlForce.cpp`), or it's fixed at construction (then don't re-pass it every call). Right now it's silently both, which is the kind of thing that produces a hard-to-explain figure in a paper ("why did increasing α in the config do nothing?").
5. **Call-site arity**: `DualArmControlForce.cpp` calls `qpOptimizer_.optimize(qpInput, f_input)` — two arguments — but the header signature requires three (`input, params, out_f_input`).
6. **Friction bound is one-step-behind and decoupled**: `computeBounds` uses the *previously measured* tangential force `fL.head<2>().norm()` to cap the *next* commanded normal force. This is a common and defensible simplification (it avoids a nonlinear/SOCP formulation — see §3.3), but it means the friction constraint is not evaluated against the force the QP is about to command, only against the last measurement. Worth stating explicitly as a design choice with a one-cycle lag, not something to discover during review.
7. **Hardcoded `xl = (-80, -80)`**: magic numbers with no path from `Params`. Fine for now, but pull into `Params` before writing the paper — reviewers will ask what -80 N means physically and whether it was tuned per-object.
8. **`DemandForces` is open-loop/scheduled, not state-feedback**: it depends on `t_norm` (trajectory phase), not on measured or estimated object velocity/acceleration. That's a legitimate design choice for a known, pre-planned trajectory (and it's smooth and easy to certify), but it means the "motion-adaptive" claim is really "trajectory-schedule-adaptive." If you want the stronger claim ("adapts to how the object is actually moving," which is the more publishable framing given the grip-force/slip literature below), you'll want a version driven by estimated `ẋ_object` or reflected-mass × acceleration, with the current version as the feedforward baseline. This is the single biggest lever for making the paper's contribution claim solid — see §4.
9. **Filter bank in `filtering()`** (3 α-β filters + 5 low-pass variants, one is literally labeled "for comparison only") reads like a tuning scratchpad, not a designed component. For the paper, pick one filter (or justify a two-stage combination), and cut the rest — or turn the comparison into a small ablation table if it's genuinely informative.

None of this is fatal — it's the normal state of research code — but items 2–5 mean the optimizer as pasted won't build, and item 8 is the one that determines how strong a claim you get to make.

---

## 2. State of the art — three literatures you're sitting at the intersection of

### 2.1 Dual-arm internal/external force decomposition and QP allocation (your closest neighbors)

This is a long-running line going back to Uchiyama & Dauchez's hybrid position/force scheme and Nahon & Angeles' QP-based internal-force minimization, with friction-cone-constrained force distribution treated via NLP/QP by Kwon & Lee and Kazerooni. It's still active:

- Recent work formalizes exactly the ambiguity your `Pint`/`S_n` decomposition runs into: the internal (nonsqueezing) load distribution from a generalized inverse of the grasp matrix is **not unique**, and there's no universally "correct" nonsqueezing solution — the choice is a design decision, which is precisely what your α/β-weighted QP objective is making. This recent analysis proposes a new framework for internal wrenches based on kinematic constraints, showing there is no unique nonsqueezing load distribution and reframing the internal/external decomposition used in cooperative multi-manipulator control. This is a very useful citation for your Section II/III: it justifies why your objective function (minimize squeeze effort + balance) is a legitimate, explicit choice rather than "the" solution.
- Admittance/QP frameworks for dual-arm internal vs. external effort regulation exist for industrial cobots: one framework regulates internal and external efforts in the cooperative task space via a hierarchical QP that prioritizes relative-motion tracking for safety during bimanual physical interaction, demonstrated on a real dual-arm mobile cobot.
- The field is still producing new HQP variants in 2025–2026 for manipulability, mobile bases, and free-floating space manipulators, which tells you the *architecture* (nullspace projection + QP allocation) is an accepted, current publication pattern, not a stale one — good news for framing.

**Takeaway:** your `Pint`, `S_n`, and 2-variable QP are a legitimate, recognizable instance of this family. Cite it to (a) show you know the decomposition is non-unique by design, and (b) place your contribution as *what you optimize for*, not *that you use a QP*.

### 2.2 Motion/load-adaptive grip-force scaling (this is where your "task on demand" idea actually lives, and it's hot right now)

This is the literature that makes your "increase force only while moving" idea legible as a contribution, and it's surprisingly recent:

- The foundational empirical result, still cited in nearly every paper on this topic: grip force is coupled to load force with an approximately constant ratio, and includes a safety margin against slip that depends on the friction condition (Westling & Johansson, *Exp Brain Res*, 1984) — this is the biological analogue of your `DemandForces` term and is worth one sentence in your introduction as motivation, not as a method you're implementing.
- Directly on point, published in 2025: robotic slip control has traditionally relied on increasing grip force in response to detected or predicted slip, but trajectory/motion modulation is an underexplored alternative, motivated by evidence that humans use hand-acceleration adaptation alongside grip-force modulation for slip prevention (Nature Machine Intelligence, 2025). This paper is essentially arguing the flip side of what you're doing — they modulate *motion* to reduce required force; you modulate *force* in anticipation of motion. Cite it and explicitly contrast: your framework treats grip force as the adaptive variable and trajectory as given, and you should say so.
- A 2024 human-subjects study makes the same point even more directly relevant to your `t_norm`-shaped feedforward: participants modulated hand velocity and acceleration, not just grip force, to stabilize objects and prevent slip, challenging the assumption that grip force is the only effective slip-control lever. Good for framing why a *velocity/acceleration-driven* demand term (rather than a fixed trajectory-phase schedule) would be the stronger version of your idea.
- Very recent (2026) and close in spirit to your QP: reactive uniform grasp-force increases upon slip detection work for simple parallel-jaw grippers, but for multi-contact hands such uniform increases introduce unwanted object-level wrenches and complicate recovery — motivating internal-force-aware optimization instead of naive force scaling. This is almost exactly the argument for why your QP-based *allocation* (rather than just scaling both arms' forces uniformly) matters, and it's a strong, current citation to build your motivation paragraph around.

**Takeaway:** the "increase grasp force only when the object is moving" idea is well-precedented conceptually (grip-force/load-force coupling, slip literature) but the specific mechanism — trajectory-phase feedforward inside a *dual-arm cooperative QP allocator* — is, as far as this search shows, not yet published as a unit. That's your paper's actual novelty claim. State it that precisely; don't oversell "adaptive grip force" as new in general, since it isn't.

### 2.3 Impact-aware contact and reflected-mass contact establishment (your half-working piece)

This is a well-developed, active, and fairly demanding sub-field — useful to know so you calibrate how much more work your second component needs before it's a standalone contribution:

- There is a dedicated, active research group (Saccon et al., TU Eindhoven) building exactly "impact-aware dual-arm grasping": a control approach that intentionally uses simultaneous impacts to rapidly grasp objects, using overlapping ante- and post-impact reference vector fields coupled via impact dynamics to minimize post-impact velocity error and control effort, with a spatial task to synchronize the two arms' impact times (IFAC 2023), later extended into a full QP-based reference-spreading control framework for dual-arm manipulation with planned simultaneous impacts, published in IEEE Transactions on Robotics, 2024.
- A complementary, lighter-weight strand is closer to what you're doing with reflected mass: preemptive impact reduction combined with a smooth transition into conventional contact impedance control, using a serial combined impedance framework so the impact-reduction stage can be added on top of an existing impedance controller without discontinuities or excessive contact force during the transition (Arita et al., 2022/2023).
- The classical grounding for your `M_virtual_z = r * m_reflected_z` line is the **effective/reflected mass** concept from operational-space control (Khatib) — worth citing directly rather than treating the scaling as ad hoc.

**Takeaway:** your reflected-mass virtual-mass scaling during `BUILD_GRASP` is a sensible, cheap heuristic in the spirit of Arita's preemptive impact reduction, but it is a simplification relative to the formal reference-spreading / impact-aware QP frameworks above, which explicitly model the impact dynamics and plan for it rather than just softening impedance ahead of contact. **Don't position this as a novel impact-aware contribution in this paper.** Mention it as a supporting mechanism for grasp establishment (one sentence, one citation to Arita/Khatib), and save the actual impact-aware contribution for a second paper once it's further along — trying to publish it half-working alongside your QP force optimizer will drag the review of your stronger contribution down with it.

---

## 3. What to actually cite (working list)

**Internal force decomposition / dual-arm QP allocation**
- Uchiyama, M., Dauchez, P. — classical hybrid position/force scheme for two-arm coordination (foundational, cite for the internal/external decomposition concept).
- Nahon, M., Angeles, J. (1992) — QP-based minimization of internal forces in cooperating manipulators.
- Kwon, W., Lee, B.H. — NLP/QP force-distribution scheme with normal-force constraints.
- Non-uniqueness of nonsqueezing load distributions in cooperative manipulation, recent analysis (ResearchGate/Springer 2023) — cite for justifying your objective choice. 
- Admittance-QP dual-arm cobot framework (ScienceDirect / BAZAR, 2017–2018) regulating internal/external efforts. 

**Grasp/contact force optimization under friction**
- Buss, M., Hashimoto, H., Moore, J.B. (1996), *Dextrous Hand Grasping Force Optimization*, IEEE T-RA — canonical friction-cone-as-SDP result; cite as the "exact" formulation you're deliberately simplifying away from.
- Cheng, Kerr & Roth-style polyhedral/linear approximations of the friction cone — cite as precedent for your linear bound.
- A 2026 paper explicitly discussing the SOCP-vs-QP tradeoff for friction cones in real-time grasp force control — directly supports your framing of choosing a cheap QP over an exact SOCP for real-time performance. 

**Motion-adaptive / slip-driven grip force**
- Westling, G., Johansson, R.S. (1984), *Factors influencing the force control during precision grip*, Exp Brain Res 53:277–284 — one-line biological motivation. 
- Nature Machine Intelligence (2025), trajectory modulation vs. grip-force modulation for slip control — contrast/related work. 
- Human slip-control study on hand-acceleration modulation (2024) — motivates a velocity/acceleration-driven (rather than schedule-driven) version of your demand term. 
- Reactive slip control via internal-force optimization for multifingered hands (2026 arXiv) — closest conceptual neighbor to your "don't just scale force uniformly, allocate it" argument. 

**Contact establishment / impact-awareness (background only, not a core claim)**
- van Steen, Coşgun, van de Wouw, Saccon — Dual-Arm Impact-Aware Grasping through Time-Invariant Reference Spreading Control, IFAC-PapersOnLine 2023. 
- van Steen et al. — QP-based Reference Spreading Control for Dual-Arm Manipulation with Planned Simultaneous Impacts, IEEE T-RO 2024.
- Arita, Nakamura, Fujiki, Tahara — Smoothly Connected Preemptive Impact Reduction and Contact Impedance Control, 2022/2023. 
- Khatib, O. — effective/reflected mass in operational-space control (for your `computeReflectedMassZ` grounding).
- Wang, Y., Dehio, N., Tanguy, A., Kheddar, A. — Impact-aware task-space quadratic-programming control, IJRR 2023 (mentioned as related-work anchor in your future-work paragraph).

---

## 4. Is a paper possible? Yes — with a scoped claim

**Recommendation: scope this paper narrowly around Contribution A (motion-adaptive internal-force QP allocation), and demote the impact-aware contact establishment to a one-paragraph implementation detail, not a contribution.** Trying to sell both as novel will weaken the one that's actually ready.

Two ways to strengthen the core claim before submission, roughly in order of effort:

1. **Cheap, high value:** fix the bugs in §1.1, especially making `Params` flow consistently, and expose `xl`/friction margin as configurable — reviewers will poke at exactly these knobs.
2. **Medium effort, biggest payoff:** replace or augment `DemandForces(t_norm)` with a version driven by estimated object velocity/acceleration (you likely already have `spd`/reflected-mass machinery to build this from). This converts your claim from "force scales with a pre-planned trajectory schedule" to "force scales with how the object is actually moving," which lines up with the 2024–2026 slip-control literature above and is a much stronger, more general contribution.
3. **If time allows:** a small ablation — constant grasp force vs. your motion-triggered demand — on the same trajectory, reporting internal-force tracking error, peak/RMS slip proxy (or measured tangential force), and energy/effort spent squeezing. That's your Figure 1 result.

**Target venue:** given the scope (single mechanism, mc_rtc/xArm7 real hardware or high-fidelity sim, one clear comparison), this is a good fit for **IEEE RA-L (with ICRA/IROS presentation option)** or a direct **IROS/Humanoids** submission rather than a journal — the contribution is focused enough for a letter-length paper, and the venue expects exactly this kind of "one clean mechanism + hardware validation" paper.

---

## 5. Suggested paper outline

**Working title:** *"Motion-Adaptive Internal Force Allocation for Dual-Arm Cooperative Manipulation"*
(alt: *"Task-on-Demand Grasp Force Optimization for Dual-Arm Object Transport"*)

### I. Introduction
- Dual-arm cooperative manipulation needs enough squeeze force to prevent slip, but excess squeeze force is wasted effort / risks damaging or destabilizing fragile objects.
- Human grip-force control scales force with load/motion, not statically (one-sentence cite: Westling & Johansson 1984).
- Most cooperative-manipulation force-distribution work optimizes internal force *statically or per-instant*; most motion-adaptive grip-force work is single-gripper/slip-triggered, not posed as a dual-arm allocation problem.
- **Gap:** no existing framework couples (a) nullspace-based internal/external decomposition, (b) real-time QP allocation between two arms, and (c) a motion-derived force demand that ramps up specifically during object transport and relaxes at rest.
- Contributions (bullet list):
  - A QP formulation for real-time squeeze-force allocation between two cooperating arms with friction-aware bounds.
  - A motion-adaptive demand term that couples the desired internal force level to the object's trajectory state.
  - Hardware/sim validation on a dual xArm7 platform (`mc_rtc`).

### II. Related Work
- II-A. Internal/external force decomposition and QP-based load distribution in dual-arm systems.
- II-B. Friction-cone-constrained grasp/contact force optimization (exact SOCP vs. linear/QP approximations).
- II-C. Motion- and slip-adaptive grip force control (bio-inspired and robotic).
- II-D. (brief) Impact-aware contact establishment — cited as context for your grasp-initiation phase, explicitly *not* claimed as a contribution here.

### III. System Overview and Problem Formulation
- FSM overview (Idle → Independent → Collaborative: Build-Grasp → Trajectory → Cooperative Motion) — one figure.
- Grasp matrix `G`, nullspace projector `Pint = I - G⁺G`, internal wrench definition.
- Definition of the squeeze direction `n_squeeze` and its projection through `Pint`.
- Problem statement: find per-arm normal forces `x = [F_L, F_R]` that track a desired internal force level while respecting friction and actuation bounds, at every control cycle.

### IV. Motion-Adaptive Force Demand ("Task on Demand")
- Definition of `F_demand(t_norm)` and its physical interpretation (feedforward margin that peaks during transport, vanishes at rest/at trajectory endpoints).
- Design rationale vs. constant-safety-margin baselines; relation to human grip-force/load-force coupling.
- **(If implemented per §4.2)**: extension to a velocity/acceleration-driven demand term, and comparison with the trajectory-phase-scheduled version.

### V. QP-Based Squeeze Force Optimization
- Derivation of `H`, `g` from `n_squeeze`, `Pint`, `S_n`, `w_fixed` (as in `buildQPProblem`).
- Role of `α` (minimization gain) vs. `β` (inter-arm balance gain) — what each term trades off, with a small sensitivity figure/table.
- Friction-aware bound construction (`computeBounds`) and explicit discussion of the linearized, one-cycle-lagged approximation vs. an exact SOCP friction cone — cite Buss/Hashimoto/Moore and the 2026 SOCP-vs-QP paper here.
- Real-time solve (QLD), computational cost note (2-variable QP — should be near-instant; report actual solve time).

### VI. Grasp Establishment (brief, supporting-only)
- One paragraph + one figure: reflected-mass-scaled virtual mass during `BUILD_GRASP` to soften first contact.
- Explicitly framed as adopting an existing idea (reflected mass / preemptive impedance softening — cite Khatib, Arita), not as a contribution of this paper.
- State current limitation honestly (e.g., "this is a heuristic softening term; a full impact-aware treatment as in [van Steen et al., Wang et al.] is left as future work").

### VII. Experimental Validation
- Platform: dual xArm7, `mc_rtc`.
- Protocol: pick-carry-place trajectory, with/without motion-adaptive demand term (ablation), possibly varying object mass/friction.
- Metrics: internal force tracking error vs. desired `λ_desired`; peak/RMS tangential force relative to friction bound (slip margin proxy); total squeeze effort (∫F dt) as an efficiency metric; success/failure under a perturbation or higher-speed trajectory.
- Figures: (1) FSM/system diagram, (2) internal force vs. trajectory phase for both conditions, (3) friction-margin utilization, (4) solve-time histogram.

### VIII. Discussion and Limitations
- Nonuniqueness of the internal/external decomposition (cite §3) — your α/β weighting is a design choice, not "the" optimum; discuss what it privileges.
- Linearized, lagged friction bound vs. exact cone — when it could fail (fast tangential force changes).
- Trajectory-phase-scheduled demand vs. true velocity-feedback demand — generalizability limits if you keep the scheduled version.
- Grasp-establishment heuristic is not yet a full impact-aware controller.

### IX. Conclusion and Future Work
- Summary of contribution and results.
- Future work: (1) closed-loop velocity/acceleration-driven demand, (2) exact/SOCP friction handling if solve time allows, (3) completing the impact-aware contact establishment as a follow-up paper, explicitly pointing to the reference-spreading / impact-aware QP literature as the target formalism.

---

## 6. Bottom line

- **Publishable now, with the fixes above:** the motion-adaptive QP force allocator (§IV–V of the outline). This is a real, current, well-motivated idea sitting at the intersection of two active literatures (cooperative-manipulation QP allocation, and motion/slip-adaptive grip force), and I did not find this exact combination already published.
- **Not yet publishable as its own contribution:** the reflected-mass impact-aware contact establishment. Use it as supporting context in this paper (one paragraph, honestly scoped), and treat it as the seed of a second paper once it's further developed — you'll want to engage seriously with the van Steen/Saccon reference-spreading line and Wang/Kheddar's impact-aware task-space QP control before claiming novelty there.
- **Immediate next step:** fix the `Params`-threading and signature bugs (§1.1, items 2–5) so the optimizer actually builds, then decide on the scheduled-vs-feedback demand question (§4, item 2) before you start writing — that decision changes what Section IV of the paper is allowed to claim.