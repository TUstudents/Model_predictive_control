## Chapter 7: Stability and Robustness in MPC: Ensuring Reliable Performance

*"An approximate answer to the right problem is worth a good deal more than an exact answer to an approximate problem." - John Tukey*

Model Predictive Control, with its repeated online optimization, offers an intuitive appeal for achieving good performance. However, just because we solve an optimization problem at each step doesn't automatically guarantee that the closed-loop system will be **stable** in the long run, or that it will perform well when the real process inevitably deviates from our model (i.e., in the presence of **model uncertainty**). This chapter delves into the theoretical and practical aspects of ensuring stability and robustness for MPC systems. Tukey's quote reminds us that even if our MPC solution is "optimal" for a nominal model, its real value lies in how well it performs for the "right problem" – controlling the actual, uncertain plant.

### 7.1 The Challenge: Finite Horizon vs. Infinite Horizon Performance

The core MPC strategy optimizes control actions over a *finite* prediction horizon $N_p$. However, we desire good performance (including stability) over an *infinite* time horizon as the plant operates. There's an inherent tension:

*   A control sequence that is optimal for a short finite horizon might not be optimal, or even stabilizing, if considered over a longer period. Think of a chess player making a locally brilliant move that unfortunately leads to a checkmate a few moves later.
*   Without specific design considerations, there's no inherent guarantee that the MPC optimization problem will remain feasible at every future time step, or that the system state will converge to its desired setpoint.

Therefore, specific mechanisms are often incorporated into the MPC design to ensure nominal stability (stability assuming a perfect model) and, further, robust stability (stability in the presence of bounded model uncertainty).

### 7.2 Nominal Stability of Linear MPC: The Role of Terminal Ingredients

For Linear MPC (LMPC), stability analysis is more mature. The key idea is to show that the value of the MPC objective function at each step acts as a **Lyapunov function** for the closed-loop system. A Lyapunov function $V(x)$ is a scalar function of the state $x$ that is positive definite ($V(x) > 0$ for $x \neq 0, V(0)=0$) and whose value decreases along system trajectories ($V(x_{k+1}) < V(x_k)$ for $x_k \neq 0$). If such a function exists, the system is asymptotically stable.

To make the MPC cost function a suitable Lyapunov function, common strategies involve adding **terminal ingredients**:

1.  **Terminal Cost Function ($V_f(x_{N_p})$ or $\Phi_f(x_{N_p|k})$):**
    *   An additional term is added to the MPC objective function that penalizes the predicted state at the *end* of the prediction horizon, $x_{N_p|k}$.
        $J = \sum_{j=0}^{N_p-1} (x_{k+j|k}^T Q x_{k+j|k} + u_{k+j|k}^T R u_{k+j|k}) + x_{k+N_p|k}^T P_f x_{k+N_p|k}$
        (Simplified objective for regulation to origin, $r=0$. $P_f$ is the terminal weighting matrix).
    *   This $P_f$ is often chosen as the solution to the algebraic Riccati equation (ARE) associated with an infinite-horizon Linear Quadratic Regulator (LQR) for the system $(A,B)$ with weights $(Q,R)$. The LQR cost-to-go is known to be a Lyapunov function.
    *   The terminal cost encourages the MPC to steer the state towards a region where a known stabilizing (LQR) controller can take over.

2.  **Terminal Constraint Set ($\mathcal{X}_f$):**
    *   The predicted state at the end of the horizon is constrained to lie within a specific **terminal set** $\mathcal{X}_f$:
        $x_{N_p|k} \in \mathcal{X}_f$
    *   This set $\mathcal{X}_f$ is typically chosen to be a **positively invariant set** for the system under some known stabilizing local controller (e.g., the LQR controller associated with $P_f$). A set is positively invariant if, once the state enters it, it remains within it for all future times under that local controller.
    *   Furthermore, $\mathcal{X}_f$ must be a region where the constraints are satisfied.
    *   The terminal constraint ensures that at the end of the prediction horizon, the system is in a "safe" region from which stability can be maintained.

**Proof Sketch for Stability (using terminal cost and constraint):**
Let $J_k^*$ be the optimal cost at time $k$. We want to show $J_{k+1}^* \le J_k^* - \alpha(||x_k||^2 + ||u_k||^2)$ for some $\alpha > 0$, making $J_k^*$ a Lyapunov function.
1.  At time $k$, we find an optimal sequence $u_{k|k}^*, \dots, u_{k+N_p-1|k}^*$.
2.  At time $k+1$, we can construct a *feasible* (but not necessarily optimal) control sequence by shifting the previous optimal sequence and appending a "tail" controller (e.g., the LQR feedback $u = -K_{LQR}x$) for the last step, ensuring $x_{k+1+N_p|k+1} \in \mathcal{X}_f$.
3.  It can be shown that the cost associated with this feasible sequence at $k+1$, denoted $J_{k+1}^{feas}$, satisfies $J_{k+1}^{feas} \le J_k^* - (x_k^T Q x_k + u_k^T R u_k)$ (if $P_f$ is the LQR cost).
4.  Since $J_{k+1}^* \le J_{k+1}^{feas}$ (by optimality of $J_{k+1}^*$), we get the desired decrease.

**Sufficiently Long Prediction Horizon ($N_p$):**
If $N_p$ is chosen large enough, stability can sometimes be proven even without explicit terminal costs/constraints, but this is harder to guarantee and typically requires $N_p$ to be impractically long. Terminal ingredients provide a more structured and reliable way to ensure stability with shorter horizons.

### 7.3 Feasibility and Recursive Feasibility: Can We Always Find a Solution?

Beyond stability, another critical issue is **feasibility**:

*   **Initial Feasibility:** Can the MPC optimization problem be solved at time $k=0$ from the initial state $x_0$?
*   **Recursive Feasibility:** If the problem is feasible at time $k$ and we apply $u_k^*$, will it remain feasible at time $k+1$ for the resulting state $x_{k+1}$?

Loss of feasibility means the MPC controller cannot find a control sequence that satisfies all constraints, which is a critical failure.

Terminal constraint sets ($\mathcal{X}_f$) play a key role in ensuring recursive feasibility. If $\mathcal{X}_f$ is designed such that:
1.  It's a subset of the state constraint set.
2.  For any state within $\mathcal{X}_f$, there exists a control input (e.g., the local LQR law) that keeps the state within $\mathcal{X}_f$ and satisfies input constraints.
Then, if $x_{N_p|k} \in \mathcal{X}_f$ is enforced, it helps ensure that a feasible solution can be constructed at the next step.

The region of attraction (or domain of attraction) of an MPC controller is the set of initial states for which the MPC problem is recursively feasible and the system converges to the desired setpoint. Maximizing this region is an important design goal.

### 7.4 Robust MPC Design: Handling Model Uncertainty

Nominal stability assumes the model is perfect. In reality, $x_{k+1} = A x_k + B u_k$ is only an approximation of the true plant, which might be $x_{k+1}^{true} = A_{true} x_k^{true} + B_{true} u_k + w_k^{true}$. We need MPC designs that are robust to this model-plant mismatch.

Common approaches to Robust MPC:

1.  **Min-Max MPC (Worst-Case Optimization):**
    *   The model uncertainty is assumed to belong to a bounded set (e.g., $A_{true} \in [A_{nom} \pm \Delta A]$).
    *   The MPC objective is to minimize the cost for the *worst-case* realization of uncertainty within this set:
        $\min_{\mathbf{U}_k} \max_{\text{uncertainty}} J(\mathbf{U}_k, x_k, \text{uncertainty})$
    *   **Pros:** Provides strong guarantees of performance and constraint satisfaction for any uncertainty in the defined set.
    *   **Cons:** Computationally very expensive, often intractable for real-time use as it involves solving a semi-infinite optimization problem. Can be overly conservative.

2.  **Tube-Based MPC:**
    *   A more practical and widely used approach. The idea is to steer the *nominal* system trajectory while ensuring that the *true* system trajectory remains within a "tube" or "error margin" around the nominal one.
    *   **Steps:**
        a.  **Nominal System:** Design an MPC for the nominal model $x_{k+1} = A x_k + B u_k$.
        b.  **Error Dynamics:** Analyze the dynamics of the error $e_k = x_k^{true} - x_k^{nom}$ due to disturbances and model mismatch $w_k^{uncert}$.
        c.  **Ancillary Controller:** Design a local feedback controller $u_{anc,k} = K_e e_k$ to keep the error $e_k$ bounded within a robust positively invariant set $\mathcal{E}$.
        d.  **Constraint Tightening:** The constraints for the nominal MPC are tightened. If the original output constraint is $y_{max}$, the nominal MPC aims for $y_{nom,max} = y_{max} - \delta_y$, where $\delta_y$ is an upper bound on the output error $C e_k$ when $e_k \in \mathcal{E}$.
        e.  **Actual Control:** The applied control is $u_k = u_{nom,k}^* + K_e (x_k^{true} - \hat{x}_{nom,k})$, where $u_{nom,k}^*$ is from the nominal MPC and $\hat{x}_{nom,k}$ is the nominal state estimate.
    *   **Pros:** Computationally similar to nominal MPC (solves one QP). Provides robust constraint satisfaction.
    *   **Cons:** Requires characterizing the uncertainty set and finding the error invariant set $\mathcal{E}$, which can be challenging. Can be conservative if the uncertainty bounds are loose.
    **(Figure 7.1: Illustration of Tube-Based MPC showing nominal trajectory, actual trajectory, and the error tube.)**

3.  **Explicit MPC / Multi-Parametric MPC:**
    *   For LMPC with PWA (Piecewise Affine) cost/constraints, the optimal control law $u_k^*(x_k)$ can be pre-computed offline as a PWA function of the state $x_k$.
    *   The state space is partitioned into a number of polyhedral regions, and in each region, the optimal control law is an affine function of $x_k$: $u_k^* = K_i x_k + g_i$ if $x_k \in \mathcal{R}_i$.
    *   **Pros:** Online computation is extremely fast – just a lookup to find the region and an affine calculation.
    *   **Cons:** Offline computation can be very intensive. Only feasible for systems with a small number of states (e.g., < 5-10) and inputs/outputs due to the "curse of dimensionality" (number of regions grows exponentially). Robust versions (robust explicit MPC) exist but are even more complex to compute.

4.  **Offset-Free Tracking (Revisited):**
    *   The disturbance estimation and integral action techniques discussed in Chapter 5 are crucial for robustly rejecting steady-state errors caused by constant disturbances or plant-model mismatch in gains. This ensures practical robustness for setpoint tracking.

### 7.5 Tuning Horizons ($N_p, N_c$) and Weights ($\mathbf{Q}, \mathbf{R}, \mathbf{S}$): Practical Aspects

While theory provides guidance (e.g., terminal ingredients), practical tuning significantly impacts performance, stability, and robustness.

*   **Prediction Horizon ($N_p$):**
    *   Should be long enough to capture the dominant dynamics of the system and anticipate constraint activations.
    *   Typically, $N_p$ is chosen to cover the open-loop settling time or a significant portion of it.
    *   Longer $N_p$ generally improves performance but increases computational load.
    *   *Bioreactor Link:* For slow bioreactor dynamics, $N_p$ might span several hours or even days in simulation time, though discretized into manageable steps.

*   **Control Horizon ($N_c$):**
    *   $N_c \le N_p$.
    *   A shorter $N_c$ (e.g., $N_c = 1$ to $N_p/2$) reduces the number of decision variables in the optimization, speeding up computation. After $N_c$ steps, the input is often assumed to be constant or follow a simple rule.
    *   Too short $N_c$ can lead to sluggish or overly aggressive behavior.

*   **Weighting Matrices ($\mathbf{Q}, \mathbf{R}, \mathbf{S}$):**
    *   $\mathbf{Q}$ (output error weight): Determines how aggressively the controller tries to track setpoints. Larger $\mathbf{Q}$ relative to $\mathbf{R/S}$ gives faster response but may use more control effort and be more sensitive to noise.
    *   $\mathbf{R}$ (input weight): Penalizes control effort magnitude. Larger $\mathbf{R}$ leads to gentler, less costly control actions.
    *   $\mathbf{S}$ (input rate weight): Penalizes changes in control input. Larger $\mathbf{S}$ leads to smoother control signals, reducing wear on actuators.
    *   **Tuning Heuristics:**
        *   Scaling: Normalize inputs and outputs so their typical ranges are similar (e.g., 0-1 or -1 to 1) before choosing weights.
        *   Bryson's Rule (inverse square of max acceptable deviation/input): A starting point for diagonal elements.
        *   Iterative tuning based on simulation and plant tests is almost always necessary.
        *   Focus on the trade-off between responsiveness and control effort/smoothness.

*Feynman Insight:* Tuning MPC is like tuning a musical instrument. Theory tells you how the strings and pipes work, but achieving the right sound (performance) requires careful adjustment and listening to the result. The weights are your tuning pegs for balancing different aspects of the "music" the controller plays.

*Bioreactor Link:* Ensuring stability for long batch or continuous bioreactor runs is paramount. Robustness is critical due to inherent biological variability, unmodeled metabolic shifts, and sensor noise. Terminal constraints might involve ensuring substrate levels don't drop too low at the end of the horizon, or that biomass remains in a productive state. Tube-based MPC could be used to account for uncertainty in growth rates or yield coefficients.

**This chapter has addressed the crucial concerns of stability and robustness, moving MPC from a purely theoretical optimization to a reliable control strategy. By incorporating terminal ingredients, designing for uncertainty, and carefully tuning its parameters, MPC can provide high-performance, constrained control for complex systems. The next chapter will specifically focus on Nonlinear MPC (NMPC), where these concepts of stability and robustness become even more challenging but equally vital.**

---