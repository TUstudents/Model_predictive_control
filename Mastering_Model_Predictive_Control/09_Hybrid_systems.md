## Chapter 9: Advanced Topics in NMPC and Hybrid Systems: Pushing the Boundaries

*"The important thing is not to stop questioning. Curiosity has its own reason for existing." - Albert Einstein*

Having established the fundamentals of Nonlinear Model Predictive Control (NMPC), we now venture into more advanced territories. Standard NMPC, focused on setpoint tracking, is powerful, but often real-world objectives are more nuanced. What if we want to directly optimize economic performance? How do we rigorously ensure stability for NMPC? What if our system involves discrete decisions alongside continuous dynamics? This chapter explores these frontiers: stability criteria for NMPC, the paradigm of Economic NMPC, strategies for robust NMPC, and the control of hybrid systems. Curiosity about these advanced techniques allows us to push the boundaries of what MPC can achieve.

### 9.1 Stability of Nonlinear MPC: Ensuring Long-Term Good Behavior

Proving stability for NMPC is significantly more challenging than for LMPC due to the nonlinear dynamics. However, similar concepts of using Lyapunov theory and terminal ingredients are extended.

**Key Approaches for NMPC Stability:**

1.  **Terminal Cost and Terminal Constraint Set:**
    *   Analogous to LMPC, a terminal cost $\Phi_f(\hat{x}_{k+N_p|k})$ and a terminal constraint $\hat{x}_{k+N_p|k} \in \mathcal{X}_f$ are employed.
    *   $\mathcal{X}_f$ must be a **control invariant set** for the nonlinear system under some locally stabilizing (often nonlinear) controller $u = \kappa_f(x)$. This means if $x \in \mathcal{X}_f$, then $f(x, \kappa_f(x)) \in \mathcal{X}_f$, and constraints are satisfied.
    *   The terminal cost $\Phi_f(x)$ is chosen to be a **local Lyapunov function** within $\mathcal{X}_f$ for the system under the local controller $\kappa_f(x)$.
    *   The proof of stability often relies on showing that the optimal NMPC cost function $J^*(\hat{x}_k)$ is a Lyapunov function for the closed-loop system, demonstrating a decrease at each step similar to the LMPC case.
    *   Finding suitable $\mathcal{X}_f$ and $\Phi_f$ for general nonlinear systems can be very difficult. Often, $\mathcal{X}_f$ is a small region around the target setpoint where a linearized controller (like LQR) might be stabilizing, or more advanced nonlinear control techniques are used to define them.

2.  **Quasi-Infinite Horizon NMPC (QIH-NMPC):**
    *   This approach aims to approximate the performance of an infinite-horizon NMPC problem using a finite horizon.
    *   It often involves demonstrating that as $N_p \rightarrow \infty$, the finite-horizon solution approaches the infinite-horizon one.
    *   Practical QIH-NMPC schemes might use a sufficiently long $N_p$ such that the terminal state is close to the steady-state, and the terminal cost approximates the infinite horizon cost from that point onwards.

3.  **Contractivity and Dissipativity:**
    *   These are system-theoretic concepts that can be leveraged. If the system is contractive (trajectories converge to each other) or dissipative (it "dissipates" a certain form of energy or supply rate), these properties can sometimes be used to prove stability of the NMPC loop.

**Practical Challenges:**
Verifying the conditions for NMPC stability (e.g., finding appropriate $\mathcal{X}_f, \Phi_f$) is often highly non-trivial and system-specific. Many practical NMPC implementations rely on extensive simulation and careful tuning to achieve stable behavior, sometimes without formal proof for the exact deployed controller.

### 9.2 Robust Nonlinear MPC: Handling Uncertainty in Nonlinear Systems

Model uncertainty is even more pronounced and complex to handle in NMPC than in LMPC because uncertainty can affect the system's behavior in nonlinear ways.

1.  **Challenges beyond Robust LMPC:**
    *   Uncertainty propagation through nonlinear functions is more complex than through linear ones. Simple addition of bounds (as in some tube-based LMPC) might be overly conservative or not applicable.
    *   Characterizing uncertainty sets for nonlinear systems is harder.
    *   Min-max robust NMPC becomes computationally even more formidable due to the nested optimization over nonlinear functions.

2.  **Approaches to Robust NMPC:**
    *   **Scenario-Based NMPC:** The optimization considers a finite set of "scenarios" representing different possible realizations of uncertainty (e.g., different parameter values). The controller aims to perform well across all these scenarios, perhaps by minimizing a worst-case or average-case objective. Computationally intensive as it effectively solves multiple NMPCs.
    *   **Tube-Based NMPC Extensions:** Efforts have been made to extend tube concepts to nonlinear systems, often involving robust control techniques to design the ancillary controller and estimate the error tube. This is an active research area.
    *   **Constraint Back-off / Tightening:** Similar to robust LMPC, constraints are tightened based on estimated bounds of uncertainty effects. This requires methods to estimate how uncertainty propagates through the nonlinear model, possibly using interval arithmetic, reachability analysis, or sensitivity analysis.
    *   **Adaptive NMPC (discussed later):** If uncertainty is parametric and can be estimated online, the NMPC model itself can be adapted.

### 9.3 Economic NMPC (EMPC) / Dynamic Real-Time Optimization (DRTO)

Traditional MPC focuses on tracking setpoints (e.g., keeping temperature at 25°C). However, often the ultimate goal is to optimize a direct economic measure, such as:
*   Maximizing profit or production rate.
*   Minimizing energy consumption or raw material usage.
*   Optimizing product yield or quality.

**Economic MPC (EMPC) directly incorporates such an economic objective function into the NMPC formulation, rather than just penalizing deviations from pre-defined setpoints.**

**Objective Function in EMPC:**
$J_{econ}(\mathbf{U}_k, x_k) = \Phi_{econ}(\hat{x}_{k+N_p|k}) + \sum_{j=0}^{N_p-1} L_{econ}(\hat{x}_{k+j|k}, u_{k+j|k})$
Where $L_{econ}$ and $\Phi_{econ}$ represent direct economic costs or profits.
*   *Example:* For a reactor, $L_{econ}$ might be (value of product formed) - (cost of raw materials consumed) - (cost of energy used).

**Key Differences and Challenges with EMPC:**

1.  **No Explicit Setpoint:** The "optimal" operating point is not pre-defined but is determined dynamically by the optimization based on current conditions and the economic objective. The system might operate transiently to achieve better overall economic performance over the horizon.
2.  **Objective Function Properties:** Economic objective functions are often not quadratic or positive definite with respect to a desired steady-state. This can make stability analysis more complex. Standard Lyapunov arguments for setpoint tracking MPC may not directly apply.
3.  **Average Performance vs. Asymptotic Stability:** For some EMPC formulations, the goal might be to optimize average economic performance over time, even if the system doesn't settle to a specific steady-state but operates in an economically optimal cyclic fashion.
4.  **Ensuring Nominal Stability:** Special considerations are needed to ensure that pursuing economic objectives doesn't lead to instability. This might involve:
    *   Adding "safety net" regulatory terms to the cost function (e.g., penalties for deviating too far from a known stable region).
    *   Using a Lyapunov-based EMPC approach where the economic objective is maximized subject to constraints that guarantee stability (e.g., requiring a Lyapunov function to decrease or remain bounded).
    *   Two-layer approaches: An upper DRTO layer computes economically optimal setpoints for a lower-layer (N)MPC controller.

*Bioreactor Link (EMPC):*
*   **Objective:** Maximize total grams of therapeutic protein produced over a fed-batch run, considering the cost of media and supplements.
*   **NMPC Action:** The EMPC would dynamically adjust feeding profiles throughout the batch, potentially allowing temporary deviations from "ideal" physiological setpoints if it leads to a better overall economic outcome (e.g., a slightly higher substrate concentration temporarily if it boosts productivity significantly despite higher cost).
*   This is a powerful paradigm for bioprocess optimization where complex trade-offs exist between growth, productivity, resource consumption, and batch time.

### 9.4 Learning-Based MPC / Adaptive MPC: Learning from Experience

Given the difficulty of obtaining perfect first-principles models, especially for complex systems like bioreactors, there's growing interest in MPC strategies that can learn from data to improve their models or policies online.

1.  **Adaptive MPC:**
    *   If the model structure is known but some parameters are uncertain or time-varying (e.g., $\mu_{max}$ or yield coefficients in a bioreactor model), these parameters can be estimated online (e.g., using recursive least squares, EKF/MHE for joint state-parameter estimation).
    *   The NMPC controller then uses the updated model with these adapted parameters at each step.
    *   **Challenges:** Ensuring identifiability of parameters, stability of the combined estimation-control loop ("dual control" problem where inputs should also excite for better estimation).

2.  **Learning-Based MPC (using Machine Learning):**
    *   **Model Learning:** Use ML techniques (e.g., Gaussian Processes, Neural Networks - as in Appendices A/B) to learn the process model (or parts of it, like disturbances or unmodeled dynamics) from operational data. This learned model is then used within the NMPC framework.
        *   *Gaussian Process MPC:* Uses a GP to model the system or the model error. The GP's predictive uncertainty can be used for robust constraint satisfaction (e.g., chance constraints).
    *   **Policy Learning (Reinforcement Learning - RL):** Train an RL agent to learn a control policy that directly maps states (or observations) to control actions, aiming to maximize a cumulative reward (which can be an MPC-like objective).
        *   Sometimes RL is used to approximate a complex MPC law offline, creating a faster-to-execute policy.
        *   "MPC-guided RL" or using MPC for "safe exploration" in RL are active research areas.

*Feynman Insight:* Learning-based MPC is like a driver who not only plans their route (MPC) but also learns from past trips to update their map (model learning) or even refine their driving instincts (policy learning) for better future journeys.

### 9.5 Hybrid MPC: Controlling Systems with Continuous and Discrete Behavior

Many real-world systems involve an interplay between continuous dynamics (described by ODEs/DAEs) and discrete logic or events (e.g., on/off switches, mode changes, sequential operations). These are **hybrid systems**.

**Examples:**
*   A batch reactor with discrete phases: filling, heating, reaction, cooling, emptying.
*   Systems with on/off actuators (valves, pumps).
*   Supply chains with discrete ordering decisions.
*   *Bioreactor Link:*
    *   Switching between different feed media.
    *   Inducing protein expression at a specific time or condition (an on/off decision).
    *   Implementing logical rules for alarms or corrective actions.
    *   Cell cycle progression having discrete phases.

**Modeling Hybrid Systems:**
Common frameworks include:
*   **Mixed Logical Dynamical (MLD) Systems:** Represent the hybrid system using linear dynamic equations coupled with integer variables (representing discrete states/inputs) and mixed-integer linear inequalities (representing logic).
    $x_{k+1} = A x_k + B_u u_k + B_\delta \delta_k + B_z z_k$
    $y_k = C x_k + D_u u_k + D_\delta \delta_k + D_z z_k$
    $E_u u_k + E_x x_k + E_\delta \delta_k + E_z z_k \le e$
    where $\delta_k \in \{0,1\}^p$ are binary variables and $z_k \in \mathbb{R}^q$ are auxiliary continuous variables representing products of binary and continuous variables.
*   **Piecewise Affine (PWA) Systems:** The dynamics are affine within different polyhedral regions of the state-space.

**Hybrid MPC:**
When controlling hybrid systems, the MPC optimization problem often involves both continuous decision variables (like $u_k$) and integer decision variables (like $\delta_k$).
*   If the underlying model is linear (or PWA) and the cost is quadratic, this leads to a **Mixed-Integer Quadratic Program (MIQP)**.
*   If the model is nonlinear, it becomes a **Mixed-Integer Nonlinear Program (MINLP)**.

**Solving MIQPs/MINLPs:**
These are significantly harder to solve than QPs/NLPs. Common methods include:
*   **Branch and Bound:** Systematically explores a tree of subproblems where integer variables are fixed.
*   **Outer Approximation / Logic-Based Benders Decomposition:** Iteratively solve an NLP (with integers fixed) and a MIP (to update integer choices).

Computational cost is a major concern for real-time hybrid MPC, especially MINLP. Horizon lengths are often kept short.

**This chapter has journeyed through several advanced NMPC concepts, from ensuring its stability and robustness to optimizing direct economic goals and even enabling it to learn or manage hybrid dynamics. These techniques significantly broaden the applicability and power of MPC, allowing it to tackle increasingly complex and nuanced control challenges, particularly in demanding fields like bioprocessing where both economic performance and intricate system behaviors are paramount. The subsequent chapters will showcase these capabilities through detailed case studies.**

---