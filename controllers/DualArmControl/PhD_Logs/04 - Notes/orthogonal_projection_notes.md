# Orthogonal Task-Space Projection for Dual-Arm Manipulation

This document details the mathematical formulation, pipeline, and implementation guidelines for resolving the coupling/interference failure mode between Cartesian impedance control and contact-force tracking during dual-arm manipulation. The proposed approach is designed to be **orthogonal by construction** and fully compatible with the existing `mc_rtc` quadratic programming solver, bypassing the need to rewrite the framework from scratch.

---

## 1. Problem Characterization & Root Cause of Failure

During collaborative dual-arm transport (e.g., lateral translation), the manipulators must track a desired motion trajectory while maintaining contact and regulating squeeze (internal) forces. Under a standard joint-space or end-effector task architecture, these objectives are represented by separate tasks in the task-space solver:

1. **Motion/Impedance Task:** Keeps the end-effectors tracking the object's transport path.
2. **Force/Impedance Task:** Maintains contact with the object by applying normal forces along the squeeze axis.

### The Failure Mode
During non-axial motion (e.g., lateral translation), the displacement error in the motion tracking task induces a virtual spring restoring force (due to stiffness gains) that can directly oppose the commanded contact force. For instance:
- The robot accelerates laterally to the right.
- Due to tracking lag, the left arm lags behind its reference, creating a virtual spring force pulling it to the left (away from the object).
- If this virtual spring force exceeds the commanded normal contact force, the left end-effector loses contact with the object.
- Conversely, on the right arm, the lag creates a virtual force pushing to the left (into the object), causing excessive, un-modeled squeeze forces.

Because the motion task and the force task are solved simultaneously or via simple task priorities without decoupling, their objectives **compete in the same directional subspaces**, leading to contact loss or crushing.

---

## 2. Orthogonal Projection Formulation

To prevent task interference, we mathematically decompose the task space of each end-effector into two mutually orthogonal subspaces:
- The **Force Subspace** ($\mathcal{S}_f$): Direction along which force is controlled (contact normal / squeeze direction).
- The **Motion Subspace** ($\mathcal{S}_m$): Directions along which position/orientation are controlled.

By ensuring these subspaces are orthogonal by construction, the motion control inputs will never generate forces along the squeeze axis, and the force task will never interfere with the motion tracking.

### 2.1 Contact and Task Frames
Let $\{W\}$ be the world frame, and $\{C_i\}$ be the contact frame at end-effector $i \in \{L, R\}$:
- The contact normal (pointing into the object) is chosen along the Z-axis of $\{C_i\}$, denoted by the unit vector $\hat{u}_{z,i} \in \mathbb{R}^3$.
- The tangential contact plane is spanned by the X-axis and Y-axis, denoted by $\hat{u}_{x,i}, \hat{u}_{y,i} \in \mathbb{R}^3$.

The rotation matrix from contact frame $\{C_i\}$ to world frame $\{W\}$ is:
$$ R_{c,i} = \begin{bmatrix} \hat{u}_{x,i} & \hat{u}_{y,i} & \hat{u}_{z,i} \end{bmatrix} \in \text{SO}(3) $$

### 2.2 Task Selection Matrices
We define diagonal Selection Matrices in the local contact frame $\{C_i\}$ to isolate motion and force directions:
- **Force Selection Matrix** ($\Sigma_{f} \in \mathbb{R}^{3 \times 3}$):
  $$ \Sigma_f = \text{diag}(0, 0, 1) $$
- **Motion Selection Matrix** ($\Sigma_{m} \in \mathbb{R}^{3 \times 3}$):
  $$ \Sigma_m = \text{diag}(1, 1, 0) $$

Clearly, $\Sigma_f \Sigma_m = 0$ and $\Sigma_f + \Sigma_m = I_3$, establishing mutual orthogonality.

To express these selection matrices in the world frame $\{W\}$, we perform a similarity transformation:
- **World-Frame Force Projection Matrix** ($P_{f,i} \in \mathbb{R}^{3 \times 3}$):
  $$ P_{f,i} = R_{c,i} \Sigma_f R_{c,i}^T = \hat{u}_{z,i} \hat{u}_{z,i}^T $$
- **World-Frame Motion Projection Matrix** ($P_{m,i} \in \mathbb{R}^{3 \times 3}$):
  $$ P_{m,i} = R_{c,i} \Sigma_m R_{c,i}^T = I_3 - \hat{u}_{z,i} \hat{u}_{z,i}^T $$

By properties of projection matrices:
1. Symmetric: $P_{m,i}^T = P_{m,i}$, $P_{f,i}^T = P_{f,i}$
2. Idempotent: $P_{m,i}^2 = P_{m,i}$, $P_{f,i}^2 = P_{f,i}$
3. Orthogonal: $P_{m,i} P_{f,i} = 0_{3 \times 3}$

---

## 3. Decoupled Cartesian Impedance Control

We modify the operational space Cartesian impedance control law to operate exclusively within these orthogonal subspaces.

Let $e_{p,i} = x_i - x_{d,i}$ be the position tracking error in the world frame.

### 3.1 Orthogonal Position Control (Motion Subspace)
We project the position error and velocity into the motion subspace $\mathcal{S}_m$ before computing the virtual spring-damper control force. This guarantees that the motion control law **cannot** generate virtual forces along the contact normal:

$$ e_{p,i}^m = P_{m,i} e_{p,i} $$
$$ \dot{e}_{p,i}^m = P_{m,i} \dot{e}_{p,i} $$

The corresponding decoupled control force $F_{motion,i} \in \mathbb{R}^3$ in the world frame is:
$$ F_{motion,i} = - P_{m,i} \left( K_{p,i} e_{p,i}^m + D_{p,i} \dot{e}_{p,i}^m \right) $$

where $K_{p,i}, D_{p,i} > 0$ are the standard position stiffness and damping matrices. Because of the outer projection $P_{m,i}$, $F_{motion,i}$ is mathematically guaranteed to lie entirely in the tangential plane of contact:
$$ \hat{u}_{z,i}^T F_{motion,i} = 0 $$

### 3.2 Decoupled Force Control (Force Subspace)
The force tracking loop acts exclusively along the force subspace $\mathcal{S}_f$. The desired normal contact force $F_{n,i}$ computed by our QP allocator is mapped to a world-frame force target $F_{force,i} \in \mathbb{R}^3$:

$$ F_{force,i} = P_{f,i} \left( \hat{u}_{z,i} F_{n,i} \right) = \hat{u}_{z,i} F_{n,i} $$

### 3.3 Total Combined Decoupled Control Wrench
For each end-effector, the translation force $F_{cmd,i} \in \mathbb{R}^3$ and orientation moment $\tau_{cmd,i} \in \mathbb{R}^3$ are given by:

$$ F_{cmd,i} = F_{motion,i} + F_{force,i} $$
$$ \tau_{cmd,i} = - K_{\theta,i} e_{\theta,i} - D_{\theta,i} \omega_i $$

where $e_{\theta,i}$ and $\omega_i$ are orientation errors and angular velocities, which are kept decoupled from translation.

---

## 4. `mc_rtc` QP Solver Integration (Without Code Rewriting)

The main challenge is that `mc_rtc` uses an acceleration-based Hierarchical Quadratic Programming (HQP) solver. Position, force, and contact tasks are defined as equality or inequality tasks of the form:
$$ A \ddot{q} + b \approx \ddot{x}^* $$

To implement the orthogonal task-space projection without rewriting the internal solver or task classes, we can exploit the **target acceleration / wrench formulation** of the existing `mc_rtc` `ImpedanceTask` or `ComplianceTask`.

### 4.1 Target Task formulation in `mc_rtc`
In `mc_rtc`, the tracking tasks specify a desired operational space acceleration (or wrench) which is then solved in the QP.
For an `ImpedanceTask`, the task computes a reference acceleration $\ddot{x}_r$ based on virtual stiffness $K$ and damping $D$:
$$ \ddot{x}_r = K(x_d - x) + D(\dot{x}_d - \dot{x}) $$

By modifying the target/reference inputs provided to the existing `ImpedanceTask` or `ComplianceTask` at each control loop cycle, we can enforce orthogonal separation externally:

#### Reference Target Modification (Simplest & Recommended)
Instead of feeding the raw trajectory coordinates $x_{d,i}$ and $\dot{x}_{d,i}$ directly to the task, we project the target offset into the motion subspace and add the projected offset back to the current measured position:

1. Measure current end-effector position $x_i$ and velocity $\dot{x}_i$ from encoders/odometry.
2. Calculate raw tracking error: $e_{p,i} = x_{d,trajectory,i} - x_i$.
3. Project the error onto the motion plane: $e_{p,i}^m = P_{m,i} e_{p,i}$.
4. Feed the **modified position reference** $x_{ref,i}$ and **modified velocity reference** $\dot{x}_{ref,i}$ to the `mc_rtc` task:
   $$ x_{ref,i} = x_i + e_{p,i}^m $$
   $$ \dot{x}_{ref,i} = P_{m,i} \dot{x}_{d,trajectory,i} $$

**Why this works:** The virtual spring inside the standard task will only see displacements $e_{p,i}^m$ that are strictly orthogonal to the squeeze direction. The solver will then naturally produce joint accelerations that move the end-effector laterally, with zero virtual spring action trying to pull or push along the normal axis.

---

## 5. Complete Implementation Pipeline (Journal Recipe)

To deploy this in the next-stage journal paper, follow this step-by-step pipeline:

[1. Path Planner] --> Generates trajectory x_d(t), v_d(t) | v [2. Normal Vector] --> Computes contact normal u_z,i and rotation R_c,i | v [3. Projection] --> Computes P_m,i = I - u_z,i * u_z,i^T | v [4. Ref Projector] --> Computes x_ref,i = x_i + P_m,i * (x_d,i - x_i) | Computes v_ref,i = P_m,i * v_d,i | +--------------------------------------+ | | v v [5. Motion Task (mc_rtc)] [6. Force Task (mc_rtc)] Inputs: x_ref,i, v_ref,i Input: F_target,i = u_z,i * F_opt,i Stiffness: K_motion Stiffness: 0 (pure force regulation) | | +------------------+-------------------+ | v [7. mc_rtc HQP Solver] (Computes joint accelerations/torques)


### Step 1: Extract current rotation matrix and contact normal
```cpp
Eigen::Matrix3d R_ee = robots().robot(robotIndex).bodyPosW(eeName).rotation();
Eigen::Vector3d u_z = R_ee * Eigen::Vector3d(0.0, 0.0, 1.0); // Assuming contact normal is EE Z-axis
Step 2: Compute Projection Matrix
Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d P_m = I3 - u_z * u_z.transpose();
Step 3: Project Position & Velocity Targets
Eigen::Vector3d pos_measured = robots().robot(robotIndex).bodyPosW(eeName).translation();
Eigen::Vector3d pos_err = pos_traj - pos_measured;
Eigen::Vector3d pos_err_projected = P_m * pos_err;

// Define orthogonal references
Eigen::Vector3d pos_ref = pos_measured + pos_err_projected;
Eigen::Vector3d vel_ref = P_m * vel_traj;
Step 4: Update Task Inputs
impedanceTask->targetPose(sva::PTransformd(R_traj, pos_ref));
impedanceTask->targetVelocity(sva::MotionVecd(omega_traj, vel_ref));