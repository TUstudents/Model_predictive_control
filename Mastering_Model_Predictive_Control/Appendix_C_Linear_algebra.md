## Appendix C: Review of Linear Algebra and Optimization Basics – The Mathematical Toolkit

Model Predictive Control, at its core, relies heavily on concepts from linear algebra (for representing system dynamics and predictions) and optimization theory (for computing optimal control actions). This appendix provides a brief refresher of some essential mathematical tools and concepts used throughout this course. It is not intended to be a comprehensive treatment but rather a quick reference and reminder.

### C.1 Linear Algebra Essentials

**C.1.1 Vectors and Matrices**

*   **Vector:** A column of numbers, e.g., $x \in \mathbb{R}^n$.
    *   **Transpose:** $x^T$ converts a column vector to a row vector.
*   **Matrix:** A rectangular array of numbers, e.g., $A \in \mathbb{R}^{m \times n}$.
    *   **Transpose:** $(A^T)_{ij} = A_{ji}$.
*   **Vector Norms:** Measure the "length" or "magnitude" of a vector.
    *   **Euclidean Norm (L2 Norm):** $||x||_2 = \sqrt{\sum_{i=1}^n x_i^2} = \sqrt{x^T x}$.
    *   **L1 Norm:** $||x||_1 = \sum_{i=1}^n |x_i|$.
    *   **L-infinity Norm:** $||x||_\infty = \max_i |x_i|$.
    *   **Weighted Norm:** $||x||_M = \sqrt{x^T M x}$ where $M$ is a positive definite matrix.
*   **Matrix Operations:**
    *   Addition/Subtraction: Element-wise, matrices must have same dimensions.
    *   Scalar Multiplication: $cA$.
    *   Matrix Multiplication: $C = AB$ where $C_{ij} = \sum_k A_{ik}B_{kj}$. Number of columns in $A$ must equal number of rows in $B$.
*   **Identity Matrix ($I$):** Square matrix with ones on the diagonal and zeros elsewhere. $AI = A, IB = B$.
*   **Inverse Matrix ($A^{-1}$):** For a square matrix $A$, if its inverse exists, $A A^{-1} = A^{-1} A = I$. A matrix is invertible (or non-singular) if its determinant is non-zero.
*   **Determinant ($\det(A)$ or $|A|$):** A scalar value associated with a square matrix.
*   **Trace ($\text{tr}(A)$):** Sum of the diagonal elements of a square matrix.

**C.1.2 Eigenvalues and Eigenvectors**

For a square matrix $A \in \mathbb{R}^{n \times n}$, a scalar $\lambda$ is an **eigenvalue** and a non-zero vector $v \in \mathbb{R}^n$ is its corresponding **eigenvector** if:
$Av = \lambda v$
*   Eigenvalues are roots of the characteristic polynomial $\det(A - \lambda I) = 0$.
*   Eigenvalues and eigenvectors are crucial for analyzing system stability (e.g., eigenvalues of the state matrix $A$ in discrete-time systems must be within the unit circle for stability).
*   For a symmetric matrix, eigenvalues are real, and eigenvectors corresponding to distinct eigenvalues are orthogonal.

**C.1.3 Singular Value Decomposition (SVD)**

Any real matrix $A \in \mathbb{R}^{m \times n}$ can be decomposed as:
$A = U \Sigma V^T$
Where:
*   $U \in \mathbb{R}^{m \times m}$ is an orthogonal matrix (its columns are orthonormal eigenvectors of $AA^T$).
*   $V \in \mathbb{R}^{n \times n}$ is an orthogonal matrix (its columns are orthonormal eigenvectors of $A^T A$).
*   $\Sigma \in \mathbb{R}^{m \times n}$ is a diagonal matrix with non-negative real numbers $\sigma_1 \ge \sigma_2 \ge \dots \ge \sigma_r > 0$ on its diagonal, called **singular values** ($r = \text{rank}(A)$).
*   SVD is fundamental in many areas, including system identification (e.g., N4SID), model reduction, and determining matrix rank.

**C.1.4 Positive Definite and Semidefinite Matrices**

A symmetric matrix $M \in \mathbb{R}^{n \times n}$ is:
*   **Positive Definite ($M > 0$):** if $x^T M x > 0$ for all non-zero vectors $x \in \mathbb{R}^n$.
    *   Equivalent conditions: All eigenvalues of $M$ are positive; all leading principal minors are positive.
*   **Positive Semidefinite ($M \ge 0$):** if $x^T M x \ge 0$ for all $x \in \mathbb{R}^n$.
    *   Equivalent conditions: All eigenvalues of $M$ are non-negative.
*   **Negative Definite ($M < 0$):** if $x^T M x < 0$ for all non-zero $x$. (Equivalent to $-M > 0$).
*   **Negative Semidefinite ($M \le 0$):** if $x^T M x \le 0$ for all $x$. (Equivalent to $-M \ge 0$).
*   **Significance in MPC:** Weighting matrices ($\mathbf{Q}, \mathbf{R}$) in the quadratic objective function are typically chosen to be positive semidefinite ($\mathbf{Q}$) or positive definite ($\mathbf{R}$) to ensure the objective function is convex. The Hessian matrix $\mathbf{H}_{QP}$ in a QP needs to be at least positive semidefinite for a minimum to exist.

### C.2 Optimization Basics

**C.2.1 Calculus for Optimization**

*   **Gradient ($\nabla f(x)$):** For a scalar function $f(x)$ of a vector $x \in \mathbb{R}^n$, the gradient is a vector of partial derivatives:
    $(\nabla f(x))_i = \frac{\partial f}{\partial x_i}$
    The gradient points in the direction of the steepest ascent of the function.
*   **Hessian ($\nabla^2 f(x)$ or $H_f(x)$):** For a scalar function $f(x)$, the Hessian is a symmetric matrix of second partial derivatives:
    $(H_f(x))_{ij} = \frac{\partial^2 f}{\partial x_i \partial x_j}$
    The Hessian describes the local curvature of the function.
    *   If $H_f(x) > 0$ (positive definite) at a point $x^*$ where $\nabla f(x^*) = 0$, then $x^*$ is a local minimum.
    *   If $f(x)$ is quadratic, e.g., $f(x) = \frac{1}{2} x^T H x + c^T x + d$, then $\nabla f(x) = Hx + c$ and $\nabla^2 f(x) = H$.

**C.2.2 Unconstrained Optimization**

Problem: $\min_{x \in \mathbb{R}^n} f(x)$
*   **Necessary Condition for Optimality:** If $x^*$ is a local minimum and $f$ is differentiable, then $\nabla f(x^*) = 0$.
*   **Sufficient Condition for Optimality:** If $\nabla f(x^*) = 0$ and $\nabla^2 f(x^*)$ is positive definite, then $x^*$ is a strict local minimum.
*   If $f(x)$ is convex, then $\nabla f(x^*) = 0$ is a necessary and sufficient condition for $x^*$ to be a global minimum.

**Common Algorithms for Unconstrained Optimization:**
*   **Gradient Descent:** $x_{k+1} = x_k - \alpha_k \nabla f(x_k)$, where $\alpha_k$ is a step size (learning rate).
*   **Newton's Method:** $x_{k+1} = x_k - (\nabla^2 f(x_k))^{-1} \nabla f(x_k)$. Converges quadratically near the minimum but requires computing and inverting the Hessian.
*   **Quasi-Newton Methods (e.g., BFGS, L-BFGS):** Approximate the inverse Hessian using gradient information, avoiding explicit Hessian computation. Often very effective.

**C.2.3 Constrained Optimization and KKT Conditions**

Consider the general constrained problem:
$\min_{x \in \mathbb{R}^n} f(x)$
Subject to:
$g_i(x) \le 0, \quad i = 1, \dots, M_I$
$h_j(x) = 0, \quad j = 1, \dots, M_E$

**Lagrangian Function:**
$\mathcal{L}(x, \mu, \lambda) = f(x) + \sum_{i=1}^{M_I} \mu_i g_i(x) + \sum_{j=1}^{M_E} \lambda_j h_j(x)$
Where $\mu_i \ge 0$ are Lagrange multipliers for inequality constraints (KKT multipliers) and $\lambda_j$ are Lagrange multipliers for equality constraints.

**Karush-Kuhn-Tucker (KKT) Conditions:**
If $x^*$ is a local minimum and certain regularity conditions (constraint qualifications) hold, then there exist Lagrange multipliers $\mu^* \ge 0$ and $\lambda^*$ such that the following KKT conditions are satisfied at $x^*, \mu^*, \lambda^*$:

1.  **Stationarity:** $\nabla_x \mathcal{L}(x^*, \mu^*, \lambda^*) = \nabla f(x^*) + \sum_{i=1}^{M_I} \mu_i^* \nabla g_i(x^*) + \sum_{j=1}^{M_E} \lambda_j^* \nabla h_j(x^*) = 0$
2.  **Primal Feasibility:**
    $g_i(x^*) \le 0$ for all $i=1, \dots, M_I$
    $h_j(x^*) = 0$ for all $j=1, \dots, M_E$
3.  **Dual Feasibility:** $\mu_i^* \ge 0$ for all $i=1, \dots, M_I$
4.  **Complementary Slackness:** $\mu_i^* g_i(x^*) = 0$ for all $i=1, \dots, M_I$.
    This implies that if an inequality constraint $g_i(x^*)$ is *not active* (i.e., $g_i(x^*) < 0$), then its corresponding multiplier $\mu_i^*$ must be zero. If $\mu_i^* > 0$, then the constraint must be active ($g_i(x^*) = 0$).

*   For convex problems (convex $f$, convex $g_i$, affine $h_j$) satisfying constraint qualifications, the KKT conditions are necessary and sufficient for global optimality.
*   Many constrained optimization algorithms (e.g., SQP, Interior Point Methods) are designed to find points that satisfy these KKT conditions.

**C.2.4 Convex Optimization**

A convex optimization problem has the form:
$\min f_0(x)$
Subject to:
$f_i(x) \le 0, \quad i = 1, \dots, M$
$Ax = b$
where $f_0, f_1, \dots, f_M$ are convex functions, and $Ax=b$ represents affine equality constraints.

**Key Properties:**
*   The feasible set is convex.
*   Any local minimum is a global minimum.
*   KKT conditions are sufficient for optimality.

**Important Classes of Convex Optimization Problems relevant to MPC:**
*   **Linear Program (LP):** Objective and inequality constraints are linear. $f_0(x) = c^T x$, $f_i(x) = a_i^T x - b_i$.
*   **Quadratic Program (QP):** Objective is quadratic and convex ($H \ge 0$), constraints are linear.
    $\min \frac{1}{2} x^T H x + c^T x$ subject to $Ax \le b, Gx = h$.
    (LMPC problems are QPs).

This brief overview covers some of the most essential mathematical concepts from linear algebra and optimization that are frequently encountered in the study and application of Model Predictive Control. A deeper understanding of these topics can be gained from dedicated textbooks on these subjects. Having these tools in hand allows for a more rigorous formulation and analysis of MPC algorithms.

---