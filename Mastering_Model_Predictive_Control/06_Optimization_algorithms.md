## Chapter 6: Optimization Algorithms for MPC: The Engine Room

*"To optimize is to do the best you can with what you have." - Anonymous*

At the core of every Model Predictive Control iteration lies an optimization problem. The controller meticulously predicts future system behavior and then, like a skilled strategist, searches for the best sequence of control actions that will steer the system towards its objectives while respecting all constraints. This search is performed by numerical optimization algorithms. This chapter explores the types of optimization problems encountered in MPC and the common algorithms used to solve them, focusing on Quadratic Programs (QPs) for Linear MPC and the more general Nonlinear Programs (NLPs) required for Nonlinear MPC. Understanding these algorithms is key to appreciating both the power and the computational demands of MPC.

### 6.1 Fundamentals of Mathematical Optimization: A Quick Tour

Before diving into specific algorithms, let's briefly review some fundamental concepts from mathematical optimization. An optimization problem generally takes the form:

$\min_{z \in \mathbb{R}^N} f(z)$
Subject to:
$g_i(z) \le 0, \quad i = 1, \dots, M_I \quad \text{(Inequality constraints)}$
$h_j(z) = 0, \quad j = 1, \dots, M_E \quad \text{(Equality constraints)}$

Where:
*   $z$ is the vector of decision variables (in MPC, this is $\mathbf{U}_k$, the sequence of future control inputs).
*   $f(z)$ is the objective function (or cost function) to be minimized.
*   $g_i(z)$ are inequality constraint functions.
*   $h_j(z)$ are equality constraint functions.

**Key Concepts:**

*   **Feasible Set ($\mathcal{F}$):** The set of all $z$ that satisfy all constraints. $\mathcal{F} = \{z \in \mathbb{R}^N \mid g_i(z) \le 0 \forall i, h_j(z) = 0 \forall j \}$.
*   **Local Minimum:** A point $z^*$ is a local minimum if $f(z^*) \le f(z)$ for all feasible $z$ in a neighborhood around $z^*$.
*   **Global Minimum:** A point $z^*$ is a global minimum if $f(z^*) \le f(z)$ for all feasible $z \in \mathcal{F}$.
*   **Convexity:**
    *   A set $\mathcal{F}$ is **convex** if for any $z_1, z_2 \in \mathcal{F}$ and any $\theta \in [0,1]$, the point $\theta z_1 + (1-\theta)z_2$ is also in $\mathcal{F}$ (the line segment connecting $z_1$ and $z_2$ is entirely within $\mathcal{F}$).
    *   A function $f(z)$ is **convex** if its domain is a convex set and for any $z_1, z_2$ in its domain and $\theta \in [0,1]$:
        $f(\theta z_1 + (1-\theta)z_2) \le \theta f(z_1) + (1-\theta)f(z_2)$.
        (The function lies below its secant lines).
    *   **Significance:** If the objective function $f(z)$ is convex and the feasible set $\mathcal{F}$ is convex (which occurs if $g_i(z)$ are convex and $h_j(z)$ are affine/linear), then any local minimum is also a global minimum. This is a highly desirable property!

*   **Optimality Conditions (Karush-Kuhn-Tucker - KKT conditions):**
    *   For a constrained optimization problem, the KKT conditions are necessary conditions for a solution to be optimal (under certain regularity conditions, they are also sufficient for convex problems).
    *   They involve the gradients of the objective function and constraint functions, and Lagrange multipliers associated with the constraints. These conditions essentially state that at an optimal point, no feasible direction can improve the objective function.
    *   Many optimization algorithms are designed to find points that satisfy the KKT conditions.

*   **Duality:** Often, an optimization problem (the primal problem) can be associated with another optimization problem (the dual problem). Sometimes, solving the dual problem is easier or provides useful bounds on the primal solution.

### 6.2 Quadratic Programming (QP) for Linear MPC: The Efficient Workhorse

As established in Chapter 2, for Linear MPC with a quadratic objective function and linear constraints, the optimization problem becomes a Quadratic Program (QP):

$\min_{\mathbf{U}_k} \frac{1}{2} \mathbf{U}_k^T \mathbf{H}_{QP} \mathbf{U}_k + \mathbf{f}_{QP}^T \mathbf{U}_k$
Subject to: $A_{QP} \mathbf{U}_k \le b_{QP}$

*   $\mathbf{H}_{QP}$ is a symmetric matrix (positive definite if terms like $\bar{\mathbf{R}} > 0$ or $\bar{\mathbf{S}} > 0$ are included in the cost, ensuring convexity).
*   $\mathbf{f}_{QP}$ is a vector.
*   $A_{QP}$ and $b_{QP}$ define the linear inequality constraints.

This is a **convex optimization problem** (if $\mathbf{H}_{QP} \ge 0$). Efficient and reliable algorithms exist to solve QPs:

1.  **Active Set Methods:**
    *   These methods iterate by maintaining a guess of the set of constraints that are "active" (i.e., satisfied as equalities, $A_{QP,i} \mathbf{U}_k = b_{QP,i}$) at the solution.
    *   At each iteration, an equality-constrained QP (simpler to solve) is solved using the current active set.
    *   The algorithm then checks if any inactive constraints are violated or if any Lagrange multipliers for active constraints suggest they should become inactive. The active set is updated, and the process repeats.
    *   **Pros:** Very efficient when the number of active constraints at the solution is small. Can be warm-started effectively (using the active set from the previous MPC solution as an initial guess, which is often very good due to the receding horizon).
    *   **Cons:** Can be slower if many active set changes are needed. Worst-case complexity can be exponential, though typically performs well.

2.  **Interior Point Methods (IPMs):**
    *   These methods approach the optimal solution from the *interior* of the feasible region (i.e., strictly satisfying inequality constraints).
    *   They use barrier functions (e.g., logarithmic barrier for $A_{QP} \mathbf{U}_k \le b_{QP}$) to transform the constrained problem into a sequence of unconstrained (or equality-constrained) problems.
    *   Each step involves solving a system of linear equations derived from Newton's method applied to the KKT conditions of the barrier problem.
    *   **Pros:** Polynomial-time complexity in theory and practice. Often more robust and can handle larger, more complex QPs than active set methods.
    *   **Cons:** Each iteration is generally more computationally expensive than an active set iteration. Warm-starting can be more challenging than for active set methods.

**Computational Complexity of QPs:**
The number of decision variables in the QP for LMPC is roughly $N_c \cdot m$ (control horizon times number of inputs). The number of constraints can also be significant. Solution times for QPs typically scale polynomially with these dimensions. For many LMPC applications, QPs can be solved in milliseconds to seconds, making them suitable for real-time control.

**Warm-Starting in MPC:**
Because MPC solves a similar QP at each time step $k$, the optimal solution $\mathbf{U}_{k-1}^*$ from the previous step (or a shifted version of it) is often a very good initial guess for $\mathbf{U}_k^*$. This "warm-start" can significantly speed up QP solvers, especially active set methods.

### 6.3 Nonlinear Programming (NLP) for Nonlinear MPC: Tackling Complexity

When the process model $f(x,u)$ or the output function $h(x,u)$ is nonlinear, or if the constraints or objective function are nonlinear, the MPC optimization problem becomes a Nonlinear Program (NLP):

$\min_{\mathbf{U}_k} J(\mathbf{U}_k, x_k) = \sum_{j=1}^{N_p} L_j(\hat{y}_{k+j|k}, u_{k+j-1|k}) + \Phi_f(\hat{x}_{k+N_p|k})$ (General form with stage cost $L_j$ and terminal cost $\Phi_f$)
Subject to:
$\hat{x}_{i+1|k} = f(\hat{x}_{i|k}, u_{i|k}), \quad \hat{x}_{k|k} = x_k \quad \text{(Nonlinear model equations)}$
$g_c(\hat{x}_{i|k}, u_{i|k}) \le 0 \quad \text{(Path constraints)}$
$g_f(\hat{x}_{k+N_p|k}) \le 0 \quad \text{(Terminal constraints)}$

**Challenges with NLPs:**

*   **Non-convexity:** NLPs are often non-convex. This means algorithms might converge to a local minimum that is not the global minimum. The solution found can depend on the initial guess.
*   **Computational Cost:** Solving NLPs is generally much more computationally demanding than solving QPs.
*   **Constraint Handling:** Ensuring feasibility and handling active constraints in nonlinear settings is more complex.

Common algorithms for solving NLPs in NMPC include:

1.  **Sequential Quadratic Programming (SQP):**
    *   This is one of the most successful methods for constrained NLP.
    *   At each iteration $l$:
        *   It approximates the NLP by forming a QP subproblem. This QP is created by linearizing the constraints around the current iterate $\mathbf{U}_k^{(l)}$ and forming a quadratic approximation of the Lagrangian function (which combines the objective and constraints using Lagrange multipliers).
        *   The QP subproblem is solved to find a search direction $\mathbf{p}^{(l)}$.
        *   A line search is performed along $\mathbf{p}^{(l)}$ to find the next iterate $\mathbf{U}_k^{(l+1)} = \mathbf{U}_k^{(l)} + \alpha^{(l)} \mathbf{p}^{(l)}$, ensuring improvement in a merit function (which balances objective reduction and constraint violation).
    *   **Pros:** Fast convergence (superlinear or quadratic) near a solution if certain conditions hold.
    *   **Cons:** Each iteration involves solving a QP. Requires computation of gradients (and possibly Hessians) of objective and constraint functions.

2.  **Interior Point Methods for NLP (IPOPT, etc.):**
    *   Similar in spirit to IPMs for QPs, these methods also use barrier functions to handle inequality constraints and iterate towards a solution from the interior of the feasible region.
    *   They typically solve a sequence of equality-constrained barrier problems using Newton-like steps, which involve solving large, sparse linear systems.
    *   **Pros:** Good for large-scale NLPs, robust.
    *   **Cons:** Can be computationally intensive per iteration.

**Automatic Differentiation (AD):**
Most NLP solvers require gradients (Jacobians of constraints, gradient of the objective) and sometimes Hessians. Manually deriving these for complex nonlinear models used in NMPC is tedious and error-prone. Automatic Differentiation tools (e.g., CasADi, ADOL-C, Tapenade) can automatically compute exact derivatives of functions implemented as computer code, which is a huge enabler for NMPC.

### 6.4 Discretization Methods for Continuous-Time Optimal Control (NMPC)

Many NMPC problems originate from continuous-time models ($\dot{x} = f_c(x,u)$). To solve them numerically, the continuous-time optimal control problem must be transcribed into a finite-dimensional NLP. Common approaches include:

1.  **Direct Single Shooting:**
    *   Only the control inputs $u(t)$ (parameterized, e.g., piecewise constant over $N_c$ intervals) are decision variables for the NLP.
    *   For a given sequence of control inputs $\mathbf{U}_k$, the state trajectory $\hat{x}(t)$ is obtained by integrating the nonlinear ODEs $f_c(x,u)$ forward from the initial state $x_k$.
    *   The objective and constraints are then evaluated based on this simulated trajectory.
    *   **Pros:** NLP has fewer variables.
    *   **Cons:** Integrating the ODEs can be computationally expensive within the NLP solver's iterations. Can suffer from numerical difficulties if the system dynamics are unstable or very sensitive (the "tail wags the dog" problem).

2.  **Direct Multiple Shooting:**
    *   The control inputs AND the states at multiple "shooting nodes" along the prediction horizon are treated as decision variables in the NLP.
    *   The ODE integration is performed over shorter intervals between these shooting nodes.
    *   Equality constraints are added to the NLP to ensure continuity of the state trajectory between shooting nodes: $x_{j+1}^{start} - x_j^{end}(\text{from integrating } f_c \text{ over interval } j) = 0$.
    *   **Pros:** Numerically more stable than single shooting, better for unstable systems. Allows for parallelization of ODE integrations.
    *   **Cons:** Results in a larger NLP with more variables and equality constraints, but these constraints are often sparse.

3.  **Direct Collocation Methods (e.g., Hermite-Simpson, Orthogonal Collocation on Finite Elements):**
    *   The state and control trajectories are approximated by polynomials (e.g., Lagrange polynomials) over finite elements within the prediction horizon.
    *   The decision variables are the values of states and controls at specific collocation points within each element.
    *   The differential equations are enforced as algebraic equality constraints at these collocation points (i.e., the polynomial derivative must match $f_c(x,u)$ at these points).
    *   **Pros:** Can be very accurate. Transforms the DAE/ODE system directly into a large, sparse NLP. Often the most robust method for complex problems.
    *   **Cons:** Leads to very large NLPs, but their structure can be exploited by specialized solvers.

*Feynman Insight:* Choosing a discretization method is like deciding how to approximate a smooth curve for calculation. Single shooting is like picking a few control points and drawing one long curve through them. Multiple shooting is like breaking it into segments, ensuring they meet smoothly. Collocation is like fitting polynomial pieces very precisely at many intermediate points. Each has trade-offs in complexity and accuracy.

### 6.5 Software Tools and Libraries: The Implementer's Toolkit

A variety of software tools are available to help formulate and solve MPC optimization problems:

*   **Modeling Languages/Interfaces:**
    *   **MATLAB:** Optimization Toolbox (quadprog, fmincon), MPC Toolbox (automates LMPC setup).
    *   **Python:**
        *   SciPy (`scipy.optimize.minimize` for general NLP, interfaces to QPSolvers).
        *   CVXPY, Pyomo, CasADi, GEKKO: Algebraic modeling languages that allow symbolic problem definition and interface with various solvers. CasADi is particularly strong for NMPC due to its AD capabilities.
*   **Solvers:**
    *   **QP Solvers:** OSQP, qpOASES, GUROBI, CPLEX, MOSEK.
    *   **NLP Solvers:** IPOPT (popular open-source IPM), SNOPT, KNITRO, WORHP.
    *   Many solvers are available as libraries that can be called from MATLAB, Python, C++, etc.

The choice of solver often depends on the problem size, required solution speed, licensing costs, and robustness.

### 6.6 Real-Time Feasibility and Computational Demands

The Achilles' heel of MPC can be its computational demand, especially for NMPC. The optimization problem must be solved within one sampling interval $T_s$.

*   **Factors affecting solution time:**
    *   Prediction and control horizon lengths ($N_p, N_c$).
    *   Number of states ($n$), inputs ($m$), and outputs ($p$).
    *   Complexity of the model (linear vs. nonlinear, size of ODEs).
    *   Number and type of constraints.
    *   Efficiency of the chosen solver and hardware.

*Bioreactor Link:* Bioprocesses often have relatively slow dynamics (sampling times in minutes to hours). This can be an advantage, allowing more time for complex NMPC calculations involving detailed nonlinear models. However, if very frequent control actions are needed (e.g., for fast DO dynamics), efficient solvers and model simplification might still be necessary.

**This chapter has illuminated the computational core of MPC. Whether solving a QP for LMPC or a complex NLP for NMPC, the choice of algorithm, discretization method, and software tools significantly impacts performance and real-time feasibility. With an understanding of how MPC makes its decisions, we next turn to a critical question: will these decisions consistently lead to good, stable behavior in the long run? Chapter 7 will address stability and robustness in MPC.**

---