# Summary of next Implementation Steps for a complete Framework

## 1.[GAP] Integrating Joint-Space Constraints / Objectives

Currently, the QP operates entirely at the end-effector level (minimizing end-effector squeeze force and balancing $F_{n,L}$ and $F_{n,R}$).
However, in dual-arm manipulation, end-effector forces directly map to joint torques via the Jacobians ($\tau = J^T w$). A common and highly appreciated literature gap is Joint-Space Capacity-Aware Force Allocation.

### Idea

Can we include a joint torque minimization term or joint torque limit proximity penalties/constraints directly in the 2-variable QP?
Since $w$ is affine in $x$, the joint torques $\tau$ are also affine in $x$. This means we can penalize/prevent joint torque overload in real time, shifting the squeeze/balancing forces to the arm that has more mechanical/actuation margin.

### Joint-Space Capacity-Aware Force Allocation in the QP

Since we only optimize the 2 scalar contact normal forces ($x = [F_{n,L}, F_{n,R}]^T$), we cannot fully optimize the entire joint torque space. However, we can compute how much the normal forces $x$ affect the joint torques.

- **Mathematically:** The total end-effector wrenches are $w_L = w_{fixed,L} + S_{n,L} F_{n,L}$ and $w_R = w_{fixed,R} + S_{n,R} F_{n,R}$.
- The corresponding joint torques contributed by the contact wrenches are $\tau_{c,L} = J_L^T w_L$ and $\tau_{c,R} = J_R^T w_R$.
- By substituting $w_L$ and $w_R$, the joint torques are affine functions of $x$: $\tau_{c,L}(x) = J_L^T w_{fixed,L} + J_L^T S_{n,L} F_{n,L}$ $\tau_{c,R}(x) = J_R^T w_{fixed,R} + J_R^T S_{n,R} F_{n,R}$
- We can add a Joint Torque Optimization Penalty to the QP cost function $J(x)$. Specifically, we minimize the weighted norm of the joint torques: $J_{\tau}(x) = \gamma_L |\tau_{c,L}(x)|^2 + \gamma_R |\tau_{c,R}(x)|^2$ This penalizes normal force allocations that cause high joint torques on either arm (e.g., if an arm is close to a singularity or a high-torque configuration).
- This term is completely quadratic in $x$, so we can derive its analytical Hessian and gradient contributions and add them directly to our existing $H$ and $g$ matrices! This maintains the real-time, 2-variable, sub-millisecond QP solver.

In the new cost function: $$ J = \alpha (\lambda_{grasp})^2 + \beta (F_{n,L}-F_{n,R})^2 + \gamma_L \|\tau_{c,L}(x)\|^2 + \gamma_R \|\tau_{c,R}(x)\|^2 $$
The weights $\gamma_L$ and $\gamma_R$ act as scaling dials that balance the trade-off between the three objectives:

- **Minimizing** squeeze force ($\alpha$): Avoids crushing the object.
- **Force balancing** ($\beta$): Keeps contact forces even.
- Torque optimization** ($\gamma_L, \gamma_R$): Prevents joint torque overload on each respective arm.
  
Why make them configurable/adjustable parameters?  
If we make them configurable parameters in the code (just like alpha and beta are currently in your Params struct):

- Easy Tuning: It allows you to adjust how strongly the optimizer should care about joint torques. If you set $\gamma_L = \gamma_R = 0.0$, the optimizer behaves exactly like your original formulation. If you set them to a small value (e.g., $0.01$ or $0.1$), the optimizer will slightly adjust the normal forces to protect the joints without disrupting force tracking.

- Asymmetric Capabilities: If one arm has weaker motors or is in a structurally disadvantaged configuration (e.g., more extended, close to singularity, or experiencing thermal heating), you can set its weight higher (e.g., $\gamma_L > \gamma_R$) to tell the optimizer: "favor allocating squeeze forces to the right arm because the left arm has less torque capacity right now."

- Experiment Safety: If you are testing this on the physical robots and find that joint torque minimization makes the contact force tracking too conservative, you can instantly turn it off or scale it down from your configuration/gains panel without having to recompile the C++ code.

$$ J = \alpha (\lambda_{grasp})^2 + \beta (F_{n,L}-F_{n,R})^2 + \gamma_L \|\tau_{c,L}(x)\|^2 + \gamma_R \|\tau_{c,R}(x)\|^2 $$



## 2.[GAP] Principled Resolution of the Lateral Translation Coupling

lateral translation coupling failure (stiffness vs. wrench gain conflict) and solve it via a raised-cosine activation blend.

### Idea:
A more principled approach is to formulate an orthogonal task-space projection (using Raibert-Craig style or interaction-frame coordinates) that mathematically decouples the squeeze/force direction from the lateral motion/impedance directions.

## 4. Motion-Estimated vs. Schedule-Driven "Task-on-Demand"

Currently, F_demand is calculated based on a quintic trajectory schedule (a time-based feedforward clock). This has the limitation of being decoupled from actual robot tracking/disturbances.

### Idea:

The Idea: We can implement the "motion-estimated" formulation (using actual or commanded end-effector velocities/accelerations from sensors, passed through the alpha-beta or low-pass filters already implemented in your code).

## 5. Dynamic Weighting of the Cost Function ($\alpha$ and $\beta$)

Currently, the weights $\alpha$ (minimize squeeze) and $\beta$ (balance) are fixed.

## Idea:

We can introduce a dynamic weighting scheme. For example, during high-velocity/acceleration phases, the minimization weight ($\alpha$) decreases to favor safety, or if one robot's joint torques/temperatures are high, we dynamically adjust $\beta$ or individual joint weights to shift the load.


