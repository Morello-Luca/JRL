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
$$  ||F_{t,R}|| \leq \mu_L F_{n,R} $$



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

 
---

# Implementation Guide

## Step 1 Decide the optimization variable

Set the variable in our case 

$$ 
\begin{bmatrix}
F_{n,L} \\
F_{n,R}
\end{bmatrix}
$$

so in c++

```cpp
Eigen::Vector2d x;
```

You never initialize ```x```. QLD will compute it.

## Step 2 Build the cost function

the proposed cost function:

$$ J =  \underbrace{\alpha (\lambda_{grasp})^2}_{minimum Squeeze force} +
        \underbrace{\beta (F_{n,L}-F_{n,R})^2}_{Balancing}  
         $$

but a QP solver wants:

$$ J = \frac{1}{2}x^T H x + g^T x
         $$

so the first task is to compute ```H``` and ```g```

## Step 3 Let's ignore the squeeze term first

just for the simplicity make a QP that only minimizes

$$ (F_{nL}-F_{nR})^2 $$
expand it 
$$ (F_{nL}-F_{nR})^2 = F_{nL}^2-2F_{nL}F_{nR}+F_{nR}^2 $$

to calculate the hessian matrix we do

$$
H = 
\begin{bmatrix}
a & b \\
b & c
\end{bmatrix}
$$

we develop the first term

$$ Hx = 
\begin{bmatrix}
ax_1 & bx_2 \\
bx_1 & cx_2
\end{bmatrix}
$$

$$ xHx =[x_1 x_2] 
\begin{bmatrix}
ax_1 & bx_2 \\
bx_1 & cx_2
\end{bmatrix}
=
ax_1^2 +2bx_1x_2 + cx_2^2
$$

finally we compare the coefficients from the origial expression

$$
ax_1^2 +2bx_1x_2 + cx_2^2 \\
F_{nL}^2-2F_{nL}F_{nR}+F_{nR}^2
$$

we can see immidiately $$a = 1;$$ $$b=-1;$$ $$c = 1$$
$$
H = 
\begin{bmatrix}
1 & -1 \\
-1 & 1
\end{bmatrix}
$$

## Step 4 - Bounds
Suppose

```cpp
FminL = 8N

FminR = 9N
```

```cpp
Eigen::Vector2d xl;
Eigen::Vector2d xu;

xl << FminL,
      FminR;

xu <<
std::numeric_limits<double>::infinity(),
std::numeric_limits<double>::infinity();
```

## Step 7 - Call the solver
```
solver.solve(
H,
g,
Aeq,
beq,
Aineq,
bineq,
xl,
xu);
```













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


# Implementation pipeline
In every iteration of your control loop (inside updateContactForces()), the QP workflow follows these sequential steps:


```

[1. Sensor Inputs & Trajectory] 
               │
               ▼
[2. Compute Bounds (F_min)] ────► Construct Inequality Matrices (Aineq, bineq, xl, xu)
               │
               ▼
[3. Construct Mapping Vector c] ──► Construct Objective Matrices (H, g)
               │
               ▼
[4. Solve QP with QLDSolver]
               │
               ▼
[5. Extract Optimal Forces x*] ──► Convert to Wrench & Send to Impedance Controllers

```

## Step 1: Compute Dynamic Constraints ($F_{\text{min}}$)

Before calling the solver, compute the lower bound for each contact force using your friction cone and motion profile metrics:

1. Tangential Component Extraction:

Extract tangential force norms from sensor measurements:

$$\Vert{}F_{t,L}\Vert{} = \sqrt{F_{x,L}^2 + F_{y,L}^2}, \quad \Vert{}F_{t,R}\Vert{} = \sqrt{F_{x,R}^2 + F_{y,R}^2}$$

2. Demand Force Evaluation:

Call DemandForces(K) using your current normalized trajectory time $\tau = t/T$ to get $F_{\text{demand}}$.

3. Lower Bound Thresholds:
Compute lower limits:

$$F_{\text{min}, L} = \max \left( \frac{\Vert{}F_{t,L}\Vert{}}{\mu_L}, \, F_{\text{static}} + F_{\text{demand}} \right)$$

$$F_{\text{min}, R} = \max \left( \frac{\Vert{}F_{t,R}\Vert{}}{\mu_R}, \, F_{\text{static}} + F_{\text{demand}} \right)$$

## Step 2: Build the Selection and Mapping Vectors ($S_n$ and $c$)

To construct $H$ and $g$, you need $c = \begin{bmatrix} c_L & c_R \end{bmatrix}^T = S_n^T n_{\text{squeeze}}$.

1. Extract Unit Normal Vectors:Retrieve orientation matrices ($R_L, R_R$) to express contact normals in world coordinates ($\hat{u}_L, \hat{u}_R$).

2. Assemble Selection Matrix $S_n \in \mathbb{R}^{12 \times 2}$:Place $\hat{u}_L$ in the force section of the left arm (rows 3–5) and $\hat{u}_R$ in the right arm (rows 9–11).

3. Compute $c \in \mathbb{R}^2$:Perform the matrix multiplication:

$$c = S_n^T \, n_{\text{squeeze}}$$

4. Compute Baseline Offset $\lambda_0$:Perform:

$$\lambda_0 = n_{\text{squeeze}}^T \, w_{\text{fixed}}$$

## Step 3: Construct QP Objective Matrices ($H$ and $g$)

Using the weights $\alpha$ (squeeze minimization) and $\beta$ (force balancing):

Hessian Matrix $H \in \mathbb{R}^{2 \times 2}$:

$$H(0,0) = 2(\alpha \, c_L^2 + \beta)$$

$$H(0,1) = 2(\alpha \, c_L c_R - \beta)$$

$$H(1,0) = H(0,1)$$

$$H(1,1) = 2(\alpha \, c_R^2 + \beta)$$

Gradient Vector $g \in \mathbb{R}^2$:

$$g = 2 \, \alpha \, \lambda_0 \begin{bmatrix} c_L \\ c_R \end{bmatrix}$$

## Step 4: Map Constraints into eigen-qld Format
eigen-qld expects constraints in the canonical form:

$$A_{\text{eq}} x = b_{\text{eq}}$$

$$A_{\text{ineq}} x \geq b_{\text{ineq}}$$

$$x_l \leq x \leq x_u$$

Since $F_{n,L} \geq F_{\text{min},L}$ and $F_{n,R} \geq F_{\text{min},R}$ can be expressed directly as simple variable bounds ($x_l \leq x \leq x_u$):

1. Equality Matrices ($A_{\text{eq}}, b_{\text{eq}}$):Set to empty matrices/vectors ($0 \times 2$ and $0 \times 1$), as there are no hard equality constraints.

2. Inequality Matrices ($A_{\text{ineq}}, b_{\text{ineq}}$):Set to empty matrices/vectors if lower bounds are handled by $x_l$. Alternatively, populate as $I_2 \cdot x \geq \begin{bmatrix} F_{\text{min}, L} \\ F_{\text{min}, R} \end{bmatrix}$.

3. Lower Bounds Vector ($x_l \in \mathbb{R}^2$):$$x_l = \begin{bmatrix} F_{\text{min}, L} \\ F_{\text{min}, R} \end{bmatrix}$$

4. Upper Bounds Vector ($x_u \in \mathbb{R}^2$):Set to maximum safety limits (e.g., $80.0 \text{ N}$):$$x_u = \begin{bmatrix} F_{\text{max}} \\ F_{\text{max}} \end{bmatrix}$$

## Step 5: Solve and Apply Control Commands

1. Solve QP:Pass $(H, g, A_{\text{eq}}, b_{\text{eq}}, A_{\text{ineq}}, b_{\text{ineq}}, x_l, x_u)$ to QLDSolver::solve().

2. Extract Decision Variables:If the solver succeeds, extract $x^* = \begin{bmatrix} F_{n,L}^* \\ F_{n,R}^* \end{bmatrix} = \text{solver.result()}$.

3. Reconstruct 12D Wrench Command:

Map normal forces back into the full 12D wrench vector:

$$f_{\text{input}} = S_n x^* + w_{\text{fixed}}$$

4. Transform to Local End-Effector Frames:Rotate local force and torque vectors using $R_L^T$ and $R_R^T$.

5. Send Commands:
Pass the calculated wrenches to leftImpedanceTask_->targetWrench() and rightImpedanceTask_->targetWrench().