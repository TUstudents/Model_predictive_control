## Chapter 8: Nonlinear Model Predictive Control (NMPC): Embracing Complexity

*"Everything should be made as simple as possible, but not simpler." - Albert Einstein (paraphrased)*

Linear MPC, as we've explored, is a powerful tool. However, its underlying assumption of linearity can be, as Einstein's quote suggests, an oversimplification for many real-world systems. When process dynamics are inherently and significantly nonlinear, forcing them into a linear mold can lead to suboptimal performance or even instability, especially when operating over a wide range of conditions or far from a nominal linearization point. **Nonlinear Model Predictive Control (NMPC)** directly embraces this complexity by using a nonlinear model of the process for prediction and optimization. This chapter delves into the formulation, challenges, and nuances of NMPC.

### 8.1 Motivation for NMPC: When Straight Lines Aren't Enough

The need for NMPC arises when:

1.  **Strong Nonlinearities Dominate:**
    *   Many physical, chemical, and biological processes exhibit dynamics that are fundamentally nonlinear.
        *   *Bioreactor Example:* As discussed in Chapter 4, cell growth kinetics (Monod, Haldane), product formation rates, and the influence of pH or temperature on biological rates are typically highly nonlinear. A single linear model cannot accurately capture behavior across different phases of growth (lag, exponential, stationary) or varying substrate/product concentrations.
    *   Other examples include high-purity distillation columns, pH neutralization processes, and many robotic systems.

2.  **Wide Operating Ranges:**
    *   If the process is expected to operate over a broad range of conditions, a single linear model valid around one operating point will likely become inaccurate as the system moves away from it.
    *   Gain scheduling (using multiple LMPC controllers for different regions) can be a workaround but becomes cumbersome for many variables and can have issues with switching stability. NMPC offers a more unified approach.

3.  **Performance Limitations of LMPC:**
    *   Even if an LMPC controller can stabilize a nonlinear system around a setpoint, its performance (e.g., speed of response, overshoot, constraint handling) might be significantly degraded when the system deviates from the linearization point. NMPC, by using a more accurate model, can achieve better performance.

4.  **Economic Optimization:**
    *   When the control objective is directly economic (e.g., maximizing profit, minimizing energy consumption, as in Economic NMPC - Chapter 9), the objective function itself might be nonlinear, necessitating an NLP formulation even if the underlying dynamics were approximated linearly.

*Feynman Insight:* Using LMPC for a highly nonlinear system is like trying to navigate a winding mountain road using only a map that approximates it with a few straight-line segments. You might manage for short stretches, but you'll struggle with sharp turns and elevation changes. NMPC uses a more detailed topographical map, allowing for more precise and effective navigation.

### 8.2 NMPC Problem Formulation: The NLP at its Core

The NMPC problem, solved at each sampling time $k$, involves finding an optimal sequence of future control inputs $\mathbf{U}_k = [u_{k|k}^T, \dots, u_{k+N_c-1|k}^T]^T$ by minimizing an objective function subject to the nonlinear system dynamics and constraints.

**Objective Function (General Form):**
$J(\mathbf{U}_k, x_k) = \Phi_f(\hat{x}_{k+N_p|k}) + \sum_{j=0}^{N_p-1} L_j(\hat{x}_{k+j|k}, u_{k+j|k})$
Where:
*   $\hat{x}_{k+j|k}$ is the predicted state at future time $k+j$ based on the nonlinear model and inputs up to $u_{k+j-1|k}$ (or $u_{k+j|k}$ if inputs affect cost at same time step).
*   $L_j(\cdot, \cdot)$ is the **stage cost** at time $k+j$ (e.g., penalizing deviation from setpoint $||y_{k+j|k} - r_{k+j}||^2_{\mathbf{Q}}$ and control effort $||u_{k+j|k}||^2_{\mathbf{R}}$).
*   $\Phi_f(\cdot)$ is the **terminal cost** evaluated at the end of the prediction horizon $N_p$.

**Constraints:**

1.  **Nonlinear System Dynamics (Equality Constraints):**
    $\hat{x}_{i+1|k} = f(\hat{x}_{i|k}, u_{i|k})$ for $i=k, \dots, k+N_p-1$
    $\hat{x}_{k|k} = x_k$ (current estimated state)
    The function $f(\cdot, \cdot)$ represents the discrete-time nonlinear model. If the original model is continuous-time, $\dot{x}=f_c(x,u)$, then $f$ represents the result of numerical integration of $f_c$ over one sampling period.

2.  **Output Equations:**
    $\hat{y}_{i|k} = h(\hat{x}_{i|k}, u_{i|k})$

3.  **Path Constraints (Inequality Constraints):** Applied at each step $j=0, \dots, N_p-1$ (or $N_c-1$ for inputs).
    *   Input constraints: $u_{min} \le u_{k+j|k} \le u_{max}$
    *   Input rate constraints: $\Delta u_{min} \le u_{k+j|k} - u_{k+j-1|k} \le \Delta u_{max}$
    *   State constraints: $x_{min} \le \hat{x}_{k+j|k} \le x_{max}$
    *   Output constraints: $y_{min} \le \hat{y}_{k+j|k} \le y_{max}$

4.  **Terminal Constraints (Inequality/Equality Constraints):** Applied at $j=N_p$.
    $\hat{x}_{k+N_p|k} \in \mathcal{X}_f$ (e.g., constraint to a terminal set)

This complete formulation results in a **Nonlinear Program (NLP)**, which must be solved at each sampling instant. The decision variables for the NLP are typically the sequence of control inputs $\mathbf{U}_k$. If using direct collocation or multiple shooting (as discussed in Chapter 6), the states at intermediate points also become decision variables.

### 8.3 Computational Challenges of NMPC

While powerful, NMPC presents significant computational hurdles:

1.  **Higher Computational Cost per Iteration:**
    *   Solving an NLP is generally much more time-consuming than solving a QP of comparable size. Each iteration of an NLP solver (e.g., SQP) might itself involve solving a QP, plus function evaluations of the nonlinear model, objective, and constraints, and their gradients.
    *   The complexity of the nonlinear model $f(\cdot, \cdot)$ (e.g., number of ODEs, stiffness) directly impacts solution time.

2.  **Potential for Local Minima:**
    *   Unlike convex QPs, NLPs can have multiple local minima. Standard NLP solvers typically only guarantee convergence to a local minimum, which may not be the global one.
    *   The solution found can depend on the initial guess provided to the NLP solver. Good initialization (e.g., using a shifted version of the previous optimal solution) is crucial.
    *   For some problems, this is less of an issue, but for others (e.g., highly non-convex economic objectives), it can be a significant concern.

3.  **Guaranteeing Real-Time Feasibility:**
    *   Ensuring that the NLP solver finds a feasible solution (and converges to it) within the allotted sampling time $T_s$ can be challenging.
    *   Strategies include:
        *   Using efficient NLP solvers and tailored discretization schemes.
        *   Limiting horizon lengths ($N_p, N_c$).
        *   Model reduction/simplification.
        *   Implementing fallback strategies if the solver fails to converge in time (e.g., applying the previously computed input or a safe backup control law).

### 8.4 Numerical Solution Strategies for NMPC (Revisiting from Chapter 6 with NMPC Focus)

The choice of how to transcribe the continuous-time optimal control problem into a finite-dimensional NLP is critical for NMPC.

*   **Sequential Approach (e.g., Single Shooting):**
    *   **Process:** The NLP solver proposes a control sequence $\mathbf{U}_k$. The nonlinear ODEs $\dot{x}=f_c(x,u)$ are integrated over the horizon $N_p$ using this $\mathbf{U}_k$ to get the state trajectory $\hat{x}(t)$. Then, the objective and constraints are evaluated. Gradients are computed via sensitivity equations or adjoint methods.
    *   **NMPC Context:** Often used when the model is relatively stable and well-behaved. Simpler NLP structure but can be slow if ODE integration is costly.

*   **Simultaneous Approach (e.g., Direct Collocation, Multiple Shooting):**
    *   **Process:** Both states and controls at discrete points are treated as decision variables. The ODEs are converted into algebraic equality constraints that link these variables (e.g., by requiring the derivative of a polynomial approximation of $x(t)$ to match $f_c(x,u)$ at collocation points).
    *   **NMPC Context:** Leads to much larger but sparser NLPs. Generally more robust for unstable or stiff systems. Allows for easier exploitation of NLP solver structures (e.g., sparse KKT systems in Interior Point methods). This is often the preferred method for complex NMPC applications.
        *   *Example:* In CasADi, one might define symbolic variables for states and controls at each interval, formulate the collocation constraints, and then pass the entire sparse NLP to a solver like IPOPT.

*Feynman Insight:* The simultaneous approach is like building a complex sculpture out of many small, interconnected pieces (the discretized states and controls) all at once, ensuring all connections (dynamic equations) are satisfied simultaneously. The sequential approach is like sculpting by starting at one end and carefully shaping it forward, hoping it ends up right. For complex, wiggly sculptures (nonlinear systems), the simultaneous approach often gives more control.

### 8.5 State Estimation for NMPC (Revisiting EKF, UKF, MHE with NMPC Focus)

NMPC relies just as heavily, if not more so, on accurate state estimation as LMPC. The nonlinear estimators discussed in Chapter 5 are the tools of choice:

*   **Extended Kalman Filter (EKF):** Uses Jacobians of $f(\cdot,\cdot)$ and $h(\cdot,\cdot)$ for linearization.
*   **Unscented Kalman Filter (UKF):** Uses sigma points to propagate uncertainty through the true nonlinear functions. Often preferred over EKF for better accuracy in NMPC if computationally feasible.
*   **Moving Horizon Estimation (MHE):** Solves an optimization problem to find the most likely state trajectory given past measurements and the nonlinear model. This is philosophically very consistent with NMPC, as both use online optimization with the same nonlinear model. MHE can naturally handle constraints on states (e.g., concentrations must be positive), which is very useful.

**Consistency between Model in NMPC and Estimator:**
It's crucial that the nonlinear model $f(\cdot,\cdot)$ and $h(\cdot,\cdot)$ used in the NMPC controller is consistent with the model used in the state estimator (EKF, UKF, or MHE). Using different models can lead to performance degradation or instability.

### 8.6 Example: NMPC for a Continuous Stirred Tank Reactor (CSTR) with Nonlinear Kinetics

Consider an irreversible, exothermic reaction A $\rightarrow$ B in a CSTR.
**Model:**
$\frac{dC_A}{dt} = \frac{F}{V}(C_{A,in} - C_A) - k_0 e^{-E_a/(RT)} C_A$
$\frac{dT}{dt} = \frac{F}{V}(T_{in} - T) + \frac{(-\Delta H_R)}{\rho C_p} k_0 e^{-E_a/(RT)} C_A - \frac{UA_c}{\rho C_p V}(T - T_c)$
Where:
*   $C_A$: Concentration of A (state)
*   $T$: Reactor temperature (state)
*   $C_{A,in}, T_{in}$: Inlet concentration and temperature
*   $F$: Flow rate
*   $V$: Volume
*   $k_0, E_a, R$: Arrhenius parameters
*   $-\Delta H_R$: Heat of reaction
*   $\rho, C_p$: Density, specific heat
*   $UA_c$: Heat transfer coefficient * area
*   $T_c$: Coolant temperature (manipulated variable, $u_k$)

**NMPC Objective:** Regulate $C_A$ and $T$ to desired setpoints $C_{A,sp}, T_{sp}$ by manipulating $T_c$.
**Constraints:** $T_c^{min} \le T_c \le T_c^{max}$, $T \le T_{max}^{safety}$.

**NMPC Implementation:**
1.  Discretize the ODEs (e.g., using a numerical integrator like Runge-Kutta within the NMPC for single shooting, or use collocation for a simultaneous approach).
2.  Define $N_p, N_c$, and weighting matrices $\mathbf{Q}, \mathbf{R}$.
3.  At each sampling time $k$:
    a.  Estimate current $C_A(k), T(k)$ using an EKF, UKF, or MHE based on measurements (e.g., of $T$, and perhaps infrequent $C_A$ samples).
    b.  Solve the NLP to find optimal $T_c(k|k), \dots, T_c(k+N_c-1|k)$.
    c.  Apply $T_c(k) = T_c(k|k)^*$.

*Bioreactor Link:* The CSTR example is a good analogy for a bioreactor. The Arrhenius term $k_0 e^{-E_a/(RT)}$ is a highly nonlinear function, similar to Monod kinetics $\mu_{max} S/(K_S+S)$. An NMPC for a bioreactor would involve similar steps but with the more complex biological kinetic models from Chapter 4. For example, it might optimize feed rates $F_{glc}, F_{gln}$ to track a desired cell growth profile or maximize final product concentration, subject to constraints on substrate levels and feed pump capacities.

**NMPC offers a powerful framework for controlling complex nonlinear systems by leveraging accurate models and online optimization. While computationally demanding and presenting theoretical challenges (e.g., local minima, stability proofs), its ability to handle strong nonlinearities and constraints directly makes it indispensable for many advanced applications, particularly in bioprocessing. The next chapter will explore specialized variants and advanced topics within NMPC, such as Economic NMPC and robust NMPC.**

---