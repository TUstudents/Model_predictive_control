## Chapter 2: Foundations of Linear MPC: The Workhorse

*"The first principle is that you must not fool yourself – and you are the easiest person to fool." - Richard Feynman*

In the realm of control engineering, and especially with powerful tools like MPC, it's easy to be dazzled by the potential without fully grasping the mechanics. This chapter aims to "not fool ourselves" by meticulously building the mathematical framework for Linear Model Predictive Control (LMPC). LMPC is the most widely implemented form of MPC due to its computational tractability and effectiveness for a broad range of systems that can be adequately approximated as linear around their operating points. We will dissect how predictions are made, how performance is quantified, and how constraints are incorporated, all culminating in a standard optimization problem.

### 2.1 Discrete-Time Linear State-Space Models: The Language of Prediction

MPC is almost invariably implemented on digital computers, which operate in discrete time steps. Therefore, we primarily work with discrete-time models of the system we wish to control. The linear time-invariant (LTI) state-space representation is a common and convenient choice:

$x_{k+1} = A x_k + B u_k \quad \quad (2.1a)$
$y_k = C x_k + D u_k \quad \quad (2.1b)$

Where:
*   $k \in \mathbb{Z}_{\ge 0}$ is the discrete time index.
*   $x_k \in \mathbb{R}^n$ is the **state vector** of the system at time $k$. The state encapsulates all the information about the system's past history needed to predict its future behavior, given future inputs.
*   $u_k \in \mathbb{R}^m$ is the **control input vector** at time $k$. These are the "knobs" we can adjust.
*   $y_k \in \mathbb{R}^p$ is the **output vector** at time $k$. These are the variables we want to control or monitor.
*   $A \in \mathbb{R}^{n \times n}$ is the **state matrix**.
*   $B \in \mathbb{R}^{n \times m}$ is the **input matrix**.
*   $C \in \mathbb{R}^{p \times n}$ is the **output matrix**.
*   $D \in \mathbb{R}^{p \times m}$ is the **direct feedthrough matrix** (often zero in process control applications, meaning inputs don't instantaneously affect outputs).

If the underlying process is continuous-time, described by $\dot{x}(t) = A_c x(t) + B_c u(t)$, these discrete-time matrices ($A, B$) can be obtained by exact discretization over a sampling period $T_s$:
$A = e^{A_c T_s}$
$B = \int_0^{T_s} e^{A_c \tau} B_c d\tau$

The choice of $T_s$ is critical: too long, and we lose inter-sample information and control performance; too short, and we increase computational burden unnecessarily.

### 2.2 Prediction Equations: Peering into the Future, Step by Step

At the current time $k$, we have knowledge of the current state $x_k$ (either measured directly or estimated, as we'll see in Chapter 5). Our goal is to predict the future evolution of the system's outputs $y_{k+1|k}, y_{k+2|k}, \dots, y_{k+N_p|k}$ over a **prediction horizon** of $N_p$ steps. The notation $y_{k+j|k}$ means the predicted value of $y$ at future time $k+j$, based on information available at current time $k$.

These predictions will depend on a sequence of future control inputs that we decide upon: $u_{k|k}, u_{k+1|k}, \dots, u_{k+N_c-1|k}$, where $N_c$ is the **control horizon** ($N_c \le N_p$). For simplicity in this initial derivation, let's assume for a moment that we apply control inputs for the full prediction horizon, i.e., $N_c = N_p$. We'll refine this later.

Let's predict step-by-step using equation (2.1a):

**Step 1: Predict state at $k+1$ based on $x_k$ and $u_{k|k}$**
$\hat{x}_{k+1|k} = A x_k + B u_{k|k}$
The predicted output at $k+1$ is then:
$\hat{y}_{k+1|k} = C \hat{x}_{k+1|k} + D u_{k|k} = C(A x_k + B u_{k|k}) + D u_{k|k}$
$\hat{y}_{k+1|k} = CA x_k + (CB+D) u_{k|k}$

**Step 2: Predict state at $k+2$ based on $\hat{x}_{k+1|k}$ and $u_{k+1|k}$**
$\hat{x}_{k+2|k} = A \hat{x}_{k+1|k} + B u_{k+1|k} = A(A x_k + B u_{k|k}) + B u_{k+1|k}$
$\hat{x}_{k+2|k} = A^2 x_k + AB u_{k|k} + B u_{k+1|k}$
The predicted output at $k+2$ is:
$\hat{y}_{k+2|k} = C \hat{x}_{k+2|k} + D u_{k+1|k} = C(A^2 x_k + AB u_{k|k} + B u_{k+1|k}) + D u_{k+1|k}$
$\hat{y}_{k+2|k} = CA^2 x_k + CAB u_{k|k} + (CB+D) u_{k+1|k}$

**Step $j$: Predict state at $k+j$ based on $\hat{x}_{k+j-1|k}$ and $u_{k+j-1|k}$**
In general, for $j=1, \dots, N_p$:
$\hat{x}_{k+j|k} = A^j x_k + \sum_{i=0}^{j-1} A^{j-1-i} B u_{k+i|k}$
And the output is:
$\hat{y}_{k+j|k} = C \hat{x}_{k+j|k} + D u_{k+j-1|k}$ (Assuming $D \neq 0$ and input for $y_{k+j|k}$ is $u_{k+j-1|k}$. If $D=0$, then $y_{k+j|k} = C x_{k+j|k}$)
For clarity and commonality, let's assume $D=0$ for the output prediction structure. If $D \neq 0$, the $D u_{k+j-1|k}$ term would be added to the $C x_{k+j|k}$ part. With $D=0$:
$\hat{y}_{k+j|k} = C A^j x_k + \sum_{i=0}^{j-1} C A^{j-1-i} B u_{k+i|k}$

**Compact Matrix Formulation:**
We can stack these $N_p$ predicted outputs into a single vector $\mathbf{Y}_k = [\hat{y}_{k+1|k}^T, \hat{y}_{k+2|k}^T, \dots, \hat{y}_{k+N_p|k}^T]^T$. Similarly, we stack the future control inputs into $\mathbf{U}_k = [u_{k|k}^T, u_{k+1|k}^T, \dots, u_{k+N_p-1|k}^T]^T$. (Here, we assume $N_c = N_p$ for notation ease, but the structure of $\mathbf{\Phi}$ changes if $N_c < N_p$, typically by holding $u$ constant after $N_c$ steps).

The stacked prediction equation becomes:
$\mathbf{Y}_k = \mathbf{F} x_k + \mathbf{\Phi} \mathbf{U}_k \quad \quad (2.2)$

Where:
*   $\mathbf{Y}_k \in \mathbb{R}^{N_p \cdot p}$ is the vector of predicted future outputs.
*   $\mathbf{U}_k \in \mathbb{R}^{N_p \cdot m}$ is the vector of future control inputs.
*   $\mathbf{F} \in \mathbb{R}^{(N_p \cdot p) \times n}$ is the **free response matrix**, representing the effect of the current state $x_k$ on future outputs if no control action were taken (i.e., $\mathbf{U}_k = 0$).
    $\mathbf{F} = \begin{bmatrix} CA \\ CA^2 \\ \vdots \\ CA^{N_p} \end{bmatrix}$
*   $\mathbf{\Phi} \in \mathbb{R}^{(N_p \cdot p) \times (N_p \cdot m)}$ is the **forced response matrix** (or dynamic matrix), representing the effect of future control inputs $\mathbf{U}_k$ on future outputs, assuming $x_k=0$.
    $\mathbf{\Phi} = \begin{bmatrix}
    CB & 0 & \dots & 0 \\
    CAB & CB & \dots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    CA^{N_p-1}B & CA^{N_p-2}B & \dots & CB
    \end{bmatrix}$
    (This is a block lower triangular Toeplitz matrix. If $D \neq 0$, the diagonal blocks would be $CB+D$, and $D$ would appear in the off-diagonal terms appropriately if inputs directly influenced outputs at the same time step in the model $y_k = Cx_k + Du_k$. For the common $y_k = Cx_k$, $D u_{k+j|k}$ would be added if the input $u_{k+j|k}$ affects $y_{k+j|k}$ for $D \ne 0$. Here we used $y_{k+j|k} = C x_{k+j|k} + D u_{k+j-1|k}$ for the general step $j$, but for $j=0$ the $D u_k$ term applies to $y_k$. More rigorously, if $y_{k+j|k} = C x_{k+j|k} + D u_{k+j|k}$, then $\Phi$ would have $D$ on its main diagonal block and $CB, CAB, \dots$ on lower blocks as shown.)
    Let's stick to the standard formulation where $y_{k+j|k} = C x_{k+j|k}$ after the state prediction to avoid confusion with $D$. The matrix $\Phi$ shown above corresponds to $y_k = Cx_k$. If $y_k=Cx_k+Du_k$, then the $Du_{k+i|k}$ term affects $y_{k+i|k}$. A careful construction of $\Phi$ is needed.
    For $y_{k+j|k} = C \hat{x}_{k+j|k} + D u_{k+j-1|k}$ with $D \neq 0$ is not standard for output $y_k$.
    Typically, it is $\hat{y}_{k+j|k} = C \hat{x}_{k+j|k} + D u_{k+j|k}$. Then:
    $\mathbf{\Phi} = \begin{bmatrix}
    D & 0 & \dots & 0 \\
    CB & D & \dots & 0 \\
    CAB & CB & \dots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    CA^{N_p-2}B & CA^{N_p-3}B & \dots & D \\
    CA^{N_p-1}B & CA^{N_p-2}B & \dots & CB & D
    \end{bmatrix}$
    (No, this isn't quite right either. Let's be precise.)

    If $y_{k+j|k} = C x_{k+j|k} + D u_{k+j|k}$:
    $\hat{y}_{k+1|k} = C(Ax_k+Bu_{k|k}) + Du_{k|k} = CAx_k + (CB+D)u_{k|k}$
    $\hat{y}_{k+2|k} = C(A\hat{x}_{k+1|k}+Bu_{k+1|k}) + Du_{k+1|k} = C(A(Ax_k+Bu_{k|k})+Bu_{k+1|k}) + Du_{k+1|k}$
    $= CA^2x_k + CABu_{k|k} + (CB+D)u_{k+1|k}$
    Then $\mathbf{\Phi}$ becomes:
    $\mathbf{\Phi} = \begin{bmatrix}
    CB+D & 0 & \dots & 0 \\
    CAB & CB+D & \dots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    CA^{N_p-1}B & CA^{N_p-2}B & \dots & CB+D
    \end{bmatrix}$
    This is a common form. For $D=0$, this simplifies to the previous $\mathbf{\Phi}$. Let's proceed with this general form.

**The Control Horizon ($N_c < N_p$):**
Often, we optimize control inputs only for the first $N_c$ steps, and then assume the control input is held constant (or follows some other simple policy) for the remaining $N_p - N_c$ steps of the prediction horizon. For example, $u_{k+i|k} = u_{k+N_c-1|k}$ for $i \ge N_c$. This reduces the number of decision variables in the optimization, saving computation time. The structure of $\mathbf{U}_k$ and $\mathbf{\Phi}$ would then be adjusted accordingly. For now, to keep it simple, we'll proceed as if $N_c = N_p$.

Equation (2.2) is fundamental. It linearly maps the current state $x_k$ and the sequence of future control actions $\mathbf{U}_k$ to the sequence of predicted future outputs $\mathbf{Y}_k$.

### 2.3 Objective Function Formulation: Defining "Good" Performance

Now that we can predict the future, we need a way to quantify what a "good" future looks like. This is done through an **objective function** (or cost function), $J$, which we aim to minimize. A common choice for LMPC is a quadratic objective function:

$J(\mathbf{U}_k, x_k) = \sum_{j=1}^{N_p} || \hat{y}_{k+j|k} - r_{k+j} ||^2_{\mathbf{Q}_j} + \sum_{j=0}^{N_c-1} || u_{k+j|k} ||^2_{\mathbf{R}_j} + \sum_{j=0}^{N_c-1} || \Delta u_{k+j|k} ||^2_{\mathbf{S}_j} \quad \quad (2.3)$

Let's break this down:

1.  **Tracking Error Term:** $\sum_{j=1}^{N_p} || \hat{y}_{k+j|k} - r_{k+j} ||^2_{\mathbf{Q}_j}$
    *   $r_{k+j} \in \mathbb{R}^p$ is the **reference trajectory** or setpoint for the output $y$ at future time $k+j$. This is what we *want* the output to be.
    *   $\hat{y}_{k+j|k} - r_{k+j}$ is the predicted tracking error.
    *   $||\mathbf{v}||^2_{\mathbf{M}} = \mathbf{v}^T \mathbf{M} \mathbf{v}$ denotes a weighted squared Euclidean norm.
    *   $\mathbf{Q}_j \in \mathbb{R}^{p \times p}$ are positive semi-definite weighting matrices for the tracking error at each step $j$ in the prediction horizon. Larger $\mathbf{Q}_j$ values penalize deviations from the setpoint more heavily for output $j$. Often, $\mathbf{Q}_j = \mathbf{Q}$ (constant) for all $j$.

2.  **Control Effort Term:** $\sum_{j=0}^{N_c-1} || u_{k+j|k} ||^2_{\mathbf{R}_j}$
    *   This term penalizes the magnitude of the control inputs.
    *   $\mathbf{R}_j \in \mathbb{R}^{m \times m}$ are positive definite weighting matrices. Larger $\mathbf{R}_j$ values lead to less aggressive control action (smaller input magnitudes). Often, $\mathbf{R}_j = \mathbf{R}$ (constant).

3.  **Control Move Suppression Term (Rate of Change of Control):** $\sum_{j=0}^{N_c-1} || \Delta u_{k+j|k} ||^2_{\mathbf{S}_j}$
    *   $\Delta u_{k+j|k} = u_{k+j|k} - u_{k+j-1|k}$ is the change in control input from one step to the next (with $u_{k-1|k}$ being the input applied at the previous actual step, $u_{k-1}$).
    *   This term penalizes rapid changes in control inputs, leading to smoother control action. This is often important to avoid excessive wear on actuators.
    *   $\mathbf{S}_j \in \mathbb{R}^{m \times m}$ are positive semi-definite weighting matrices. Often, $\mathbf{S}_j = \mathbf{S}$ (constant).

**Vectorized Form of the Objective Function:**
Let $\mathbf{R}_{traj} = [r_{k+1}^T, \dots, r_{k+N_p}^T]^T$ be the vector of future reference setpoints.
Let $\bar{\mathbf{Q}} = \text{diag}(\mathbf{Q}_1, \dots, \mathbf{Q}_{N_p})$, $\bar{\mathbf{R}} = \text{diag}(\mathbf{R}_0, \dots, \mathbf{R}_{N_c-1})$, and $\bar{\mathbf{S}} = \text{diag}(\mathbf{S}_0, \dots, \mathbf{S}_{N_c-1})$.
(Note: The sum for $\mathbf{U}_k$ and $\Delta \mathbf{U}_k$ often runs up to $N_c-1$, while error up to $N_p$).

The objective function can be written compactly. If we define $\mathbf{e}_k = \mathbf{Y}_k - \mathbf{R}_{traj}$, then:
$J = \mathbf{e}_k^T \bar{\mathbf{Q}} \mathbf{e}_k + \mathbf{U}_k^T \bar{\mathbf{R}} \mathbf{U}_k + \Delta \mathbf{U}_k^T \bar{\mathbf{S}} \Delta \mathbf{U}_k$
(Care must be taken with dimensions if $N_c < N_p$. For $\Delta \mathbf{U}_k$, we need to relate it to $\mathbf{U}_k$).
$\Delta u_{k|k} = u_{k|k} - u_{k-1}$
$\Delta u_{k+1|k} = u_{k+1|k} - u_{k|k}$
...
$\Delta u_{k+N_c-1|k} = u_{k+N_c-1|k} - u_{k+N_c-2|k}$
This can be written as $\Delta \mathbf{U}_k = \mathbf{T} \mathbf{U}_k - \mathbf{T}_{prev} u_{k-1}$, where $\mathbf{T}$ is a differencing matrix.

Substituting $\mathbf{Y}_k = \mathbf{F} x_k + \mathbf{\Phi} \mathbf{U}_k$ into the error term:
$J = (\mathbf{F} x_k + \mathbf{\Phi} \mathbf{U}_k - \mathbf{R}_{traj})^T \bar{\mathbf{Q}} (\mathbf{F} x_k + \mathbf{\Phi} \mathbf{U}_k - \mathbf{R}_{traj}) + \mathbf{U}_k^T \bar{\mathbf{R}} \mathbf{U}_k + (\text{term for } \Delta \mathbf{U}_k)$

After expansion and collecting terms related to $\mathbf{U}_k$, this objective function will be quadratic in $\mathbf{U}_k$:
$J = \frac{1}{2} \mathbf{U}_k^T \mathbf{H}_{QP} \mathbf{U}_k + \mathbf{f}_{QP}^T (x_k, \mathbf{R}_{traj}, u_{k-1}) \mathbf{U}_k + \text{terms not depending on } \mathbf{U}_k$
The $\frac{1}{2}$ is conventional for QP solvers.
$\mathbf{H}_{QP} = 2(\mathbf{\Phi}^T \bar{\mathbf{Q}} \mathbf{\Phi} + \bar{\mathbf{R}} + \mathbf{T}^T \bar{\mathbf{S}} \mathbf{T})$
$\mathbf{f}_{QP}^T = 2((\mathbf{F} x_k - \mathbf{R}_{traj})^T \bar{\mathbf{Q}} \mathbf{\Phi} - u_{k-1}^T \mathbf{T}_{prev}^T \bar{\mathbf{S}} \mathbf{T})$
(The exact form of $\mathbf{H}_{QP}$ and $\mathbf{f}_{QP}$ depends on the precise definition of $\Delta \mathbf{U}_k$ and whether $N_c=N_p$).

*Feynman Insight:* The weighting matrices $\mathbf{Q}, \mathbf{R}, \mathbf{S}$ are the "dials" you use to tell the MPC what you care about most. Want tight setpoint tracking? Increase $\mathbf{Q}$. Want smooth, gentle control action? Increase $\mathbf{R}$ and/or $\mathbf{S}$. Tuning these is an art, guided by process knowledge and desired performance.

### 2.4 Constraint Handling: Operating Within Bounds

A key strength of MPC is its ability to explicitly handle constraints. These are typically formulated as linear inequalities:

1.  **Input Constraints:**
    $u_{min} \le u_{k+j|k} \le u_{max}$ for $j = 0, \dots, N_c-1$.
    This can be written as:
    $\mathbf{I} \mathbf{U}_k \le \mathbf{U}_{max}$
    $-\mathbf{I} \mathbf{U}_k \le -\mathbf{U}_{min}$
    (where $\mathbf{U}_{max}$ is a stacked vector of $u_{max}$, etc.)

2.  **Input Rate Constraints (Slew Rate Constraints):**
    $\Delta u_{min} \le \Delta u_{k+j|k} \le \Delta u_{max}$ for $j = 0, \dots, N_c-1$.
    Using $\Delta \mathbf{U}_k = \mathbf{T} \mathbf{U}_k - \mathbf{T}_{prev} u_{k-1}$:
    $\mathbf{T} \mathbf{U}_k \le \Delta \mathbf{U}_{max} + \mathbf{T}_{prev} u_{k-1}$
    $-\mathbf{T} \mathbf{U}_k \le -\Delta \mathbf{U}_{min} - \mathbf{T}_{prev} u_{k-1}$

3.  **Output Constraints:**
    $y_{min} \le \hat{y}_{k+j|k} \le y_{max}$ for $j = 1, \dots, N_p$.
    Substitute $\mathbf{Y}_k = \mathbf{F} x_k + \mathbf{\Phi} \mathbf{U}_k$:
    $\mathbf{\Phi} \mathbf{U}_k \le \mathbf{Y}_{max} - \mathbf{F} x_k$
    $-\mathbf{\Phi} \mathbf{U}_k \le -\mathbf{Y}_{min} + \mathbf{F} x_k$

All these linear inequality constraints can be combined into a single matrix inequality:
$A_{QP} \mathbf{U}_k \le b_{QP}(x_k, u_{k-1}) \quad \quad (2.4)$
The matrix $A_{QP}$ depends on $\mathbf{I}, \mathbf{T}, \mathbf{\Phi}$, while the vector $b_{QP}$ depends on the constraint bounds and also on the current state $x_k$ and previous input $u_{k-1}$ (due to output constraints and input rate constraints).

### 2.5 The Quadratic Program (QP): Solving for the Optimal Moves

At each time step $k$, the LMPC controller solves the following optimization problem to find the optimal sequence of future control inputs $\mathbf{U}_k^*$:

**Minimize:**
$J(\mathbf{U}_k) = \frac{1}{2} \mathbf{U}_k^T \mathbf{H}_{QP} \mathbf{U}_k + \mathbf{f}_{QP}^T \mathbf{U}_k$
(Ignoring terms not dependent on $\mathbf{U}_k$ as they don't affect the minimizer)

**Subject to:**
$A_{QP} \mathbf{U}_k \le b_{QP}$

This is a **Quadratic Program (QP)** because the objective function is quadratic in the decision variables $\mathbf{U}_k$, and the constraints are linear in $\mathbf{U}_k$.

*   If $\mathbf{H}_{QP}$ is positive definite (which it usually is if $\bar{\mathbf{R}} > 0$ or $\bar{\mathbf{S}} > 0$ and $\mathbf{\Phi}$ has full column rank, or $\bar{\mathbf{Q}} > 0$), the QP has a unique global minimum, provided a feasible solution exists.
*   Efficient algorithms (e.g., active set methods, interior-point methods) exist to solve QPs, making LMPC computationally feasible for many real-time applications.

### 2.6 The Receding Horizon Strategy in LMPC: Plan, Act, Re-plan

Once the QP solver finds the optimal sequence $\mathbf{U}_k^* = [u_{k|k}^{*T}, u_{k+1|k}^{*T}, \dots, u_{k+N_c-1|k}^{*T}]^T$:

1.  **Implement First Control Input:** Only the very first control input vector $u_{k|k}^*$ from this sequence is actually applied to the plant:
    $u_k = u_{k|k}^* \quad \quad (2.5)$

2.  **Shift Horizon:** At the next sampling instant, $k+1$:
    *   The actual plant output $y_{k+1}$ is measured.
    *   A new state estimate $x_{k+1}$ is obtained (using $y_{k+1}$ and $u_k$).
    *   The entire prediction and optimization process (Steps 2.2 to 2.5) is repeated for a new horizon starting from $k+1$. The reference trajectory $r_{k+j}$ and constraint bounds might also be updated.

This **receding horizon** (or rolling horizon) strategy is fundamental:

*   **Feedback Incorporation:** By re-measuring/re-estimating the state at each step, MPC inherently incorporates feedback, making it robust to disturbances and model mismatch. If a disturbance pushes the system off its predicted path, the next optimization will account for this new starting point.
*   **Adaptability:** The controller continually re-optimizes based on the latest information.

*Feynman Insight:* The receding horizon strategy is like constantly checking your map and your current location while driving, then re-planning the next segment of your journey. You don't just make one grand plan at the start and stick to it blindly; you adapt. This is why MPC is often more robust in practice than open-loop optimal control strategies.

### 2.7 Illustrative Example: Double Integrator

Consider a simple discrete-time double integrator system (e.g., position control of a point mass):
$x_{k+1} = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix} x_k + \begin{bmatrix} T_s^2/2 \\ T_s \end{bmatrix} u_k$
$y_k = \begin{bmatrix} 1 & 0 \end{bmatrix} x_k$
where $x_k = [position_k, velocity_k]^T$, $u_k$ is force, $y_k$ is position, and $T_s$ is the sampling time.

We could set up an LMPC for this system:
1.  Choose $N_p, N_c$.
2.  Construct $\mathbf{F}, \mathbf{\Phi}$ based on $A, B, C$.
3.  Define $\mathbf{Q}, \mathbf{R}$ (and possibly $\mathbf{S}$) and a reference $r_{k+j}$ (e.g., move to a new position).
4.  Define constraints: $u_{min} \le u_k \le u_{max}$, perhaps $|velocity_k| \le v_{max}$ (a state constraint, implementable via output constraints if velocity is an output or by directly constraining predicted states).
5.  At each step $k$, measure/estimate $x_k$, form and solve the QP, apply $u_{k|k}^*$.

*Bioreactor Link (Conceptual):* While most bioreactors are nonlinear, imagine controlling a single, approximately linear variable, like the liquid level in a fed-batch reactor using the inlet flow rate, or the temperature using jacket coolant flow, assuming the dynamics around the operating point are locally linear. The principles of prediction, cost function, constraints, and QP solution would apply directly. The real challenge for bioreactors, which we'll tackle later, is handling their strong nonlinearities.

**This chapter has laid the mathematical foundation for LMPC. We've seen how to predict future system behavior, define what "good" behavior means through an objective function, incorporate operational limits as constraints, and combine these into a solvable QP. The receding horizon principle then turns this optimization into a practical feedback control strategy. In the next chapters, we'll explore the crucial aspect of modeling in more detail, especially for the complex systems like bioreactors, and then move towards handling nonlinearity.**

---