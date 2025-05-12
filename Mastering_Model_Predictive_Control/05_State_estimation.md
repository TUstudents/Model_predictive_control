## Chapter 5: State Estimation and Disturbance Handling in MPC: Knowing Where You Are and What's Pushing You

*"Prediction is very difficult, especially if it's about the future." - Niels Bohr (often misattributed)*

Model Predictive Control, as its name implies, heavily relies on predicting the future state of a system. However, these predictions must start from an accurate understanding of the system's *current* state. In many real-world applications, including complex **bioreactors**, not all state variables are directly or continuously measurable. Furthermore, processes are invariably affected by unmeasured disturbances and model inaccuracies that can push the system away from its predicted trajectory. This chapter explores how MPC addresses these challenges through **state estimation** – figuring out "where the system is" – and **disturbance handling** – compensating for the "pushes" from the unknown.

### 5.1 The Need for State Estimation: When You Can't See Everything

The MPC formulation, whether linear (Chapter 2) or nonlinear (to be detailed for NMPC), typically assumes that the current state vector $x_k$ is known at each control interval $k$. This state $x_k$ serves as the initial condition for the prediction model $\hat{x}_{k+j|k} = f( \hat{x}_{k+j-1|k}, u_{k+j-1|k} )$.

However, in practice:

1.  **Unmeasured States:** Some state variables might be difficult, expensive, or impossible to measure directly online.
    *   *Bioreactor Example:* While temperature, pH, and Dissolved Oxygen (DO) might be measured online, key states like viable cell density ($X_v$), substrate concentrations ($S_{glc}, S_{gln}$), product concentration ($P_{mab}$), and intracellular metabolite levels are often only available through infrequent, time-consuming offline laboratory assays. MPC needs estimates of these between samples.
2.  **Noisy Measurements:** Even when sensors are available, their readings ($y_m$) are corrupted by measurement noise ($v_k$). Using noisy measurements directly as states can lead to erratic control actions.
    $y_{m,k} = y_k + v_k = C x_k + D u_k + v_k$
3.  **Model-Reality Mismatch:** If the model used for MPC differs from the true plant, simply propagating the model equations forward from a previous state will lead to accumulating errors. State estimation helps to "re-synchronize" the model with reality using available measurements.

Therefore, a **state estimator** (also called an **observer**) is often a critical component of an MPC implementation. Its role is to provide the best possible estimate of the current state, $\hat{x}_{k|k}$, based on:
*   The process model.
*   Past control inputs ($u_0, \dots, u_{k-1}$).
*   Past and current measurements ($y_{m,0}, \dots, y_{m,k}$).

This estimated state $\hat{x}_{k|k}$ is then used as the $x_k$ in the MPC's prediction and optimization steps. This is known as **output feedback MPC**.

### 5.2 Observers for Linear Systems: The Luenberger Observer

For linear time-invariant (LTI) systems, the Luenberger observer is a well-established method for state estimation. It essentially runs a copy of the system model in parallel with the actual process and uses the discrepancy between the measured output and the model's predicted output to correct the model's state.

The Luenberger observer dynamics are given by:
$\hat{x}_{k+1|k} = A \hat{x}_{k|k-1} + B u_k \quad \quad \text{(Prediction step - a priori estimate)}$
$\hat{x}_{k|k} = \hat{x}_{k|k-1} + L (y_{m,k} - \hat{y}_{k|k-1}) \quad \quad \text{(Correction step - a posteriori estimate)}$
where $\hat{y}_{k|k-1} = C \hat{x}_{k|k-1} + D u_k$ is the predicted output based on the a priori state estimate.
Substituting the prediction into the correction:
$\hat{x}_{k+1|k+1} = A \hat{x}_{k|k} + B u_k + L (y_{m,k+1} - (C(A\hat{x}_{k|k} + Bu_k) + Du_{k+1}))$ - this is getting a bit muddled.

Let's use standard notation for the observer estimating $\hat{x}_k$ at time $k$:
The observer runs a model:
$\dot{\hat{x}}(t) = A\hat{x}(t) + Bu(t) + L(y_m(t) - \hat{y}(t))$
$\hat{y}(t) = C\hat{x}(t) + Du(t)$
In discrete time:
$\hat{x}_{k+1|k} = A \hat{x}_{k|k} + B u_k \quad \quad \text{(Time update / Prediction of next state)}$
$\hat{x}_{k+1|k+1} = \hat{x}_{k+1|k} + L (y_{m,k+1} - (C\hat{x}_{k+1|k} + Du_{k+1})) \quad \quad \text{(Measurement update / Correction)}$

Here, $L \in \mathbb{R}^{n \times p}$ is the **observer gain matrix**. The term $(y_{m,k} - \hat{y}_{k|k-1})$ is the innovation or prediction error. The gain $L$ determines how much this error is used to correct the state estimate.

The estimation error dynamics are given by $e_{k+1} = (A-LC)e_k$, where $e_k = x_k - \hat{x}_{k|k}$. If the system $(A,C)$ is **observable** (meaning all states can be inferred from the outputs), then $L$ can be chosen such that the eigenvalues of $(A-LC)$ are stable and lie within the unit circle, ensuring that the estimation error $e_k \rightarrow 0$ asymptotically. The poles of the observer can be placed arbitrarily (pole placement) to achieve a desired convergence speed, balancing responsiveness against noise sensitivity.

### 5.3 Kalman Filtering (KF): Optimal Estimation under Gaussian Noise

The Kalman Filter is a recursive algorithm that provides an optimal estimate of the state of a linear dynamic system perturbed by Gaussian white noise, using measurements that are also corrupted by Gaussian white noise. It's "optimal" in the sense that it minimizes the mean square error of the state estimate.

Consider the LTI system with process noise $w_k$ and measurement noise $v_k$:
$x_{k+1} = A x_k + B u_k + G w_k \quad \quad (w_k \sim \mathcal{N}(0, Q_K))$
$y_{m,k} = C x_k + D u_k + v_k \quad \quad (v_k \sim \mathcal{N}(0, R_K))$
Here, $w_k$ and $v_k$ are assumed to be zero-mean, uncorrelated white noise sequences with covariance matrices $Q_K$ (process noise covariance) and $R_K$ (measurement noise covariance) respectively. $G$ is the process noise input matrix.

The Kalman Filter operates in two steps:

1.  **Time Update (Prediction):**
    *   Project the state estimate forward: $\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_{k-1}$
    *   Project the error covariance forward: $P_{k|k-1} = A P_{k-1|k-1} A^T + G Q_K G^T$
    ($P_{k|k-1}$ is the covariance of the a priori state estimate $\hat{x}_{k|k-1}$)

2.  **Measurement Update (Correction):**
    *   Compute the Kalman gain: $K_k = P_{k|k-1} C^T (C P_{k|k-1} C^T + R_K)^{-1}$
    *   Update the state estimate with measurement $y_{m,k}$: $\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (y_{m,k} - (C \hat{x}_{k|k-1} + D u_k))$
    *   Update the error covariance: $P_{k|k} = (I - K_k C) P_{k|k-1}$
    ($P_{k|k}$ is the covariance of the a posteriori state estimate $\hat{x}_{k|k}$)

The Kalman gain $K_k$ is time-varying and adapts based on the relative confidence in the model predictions (via $P_{k|k-1}$) and the measurements (via $R_K$). If measurements are very noisy (large $R_K$), $K_k$ will be small, and the filter will trust the model prediction more. If the model is uncertain (large $Q_K \Rightarrow$ large $P_{k|k-1}$), $K_k$ will be larger, and the filter will rely more on the measurements. For LTI systems, $K_k$ often converges to a steady-state value $K_\infty$.

Tuning $Q_K$ and $R_K$ is crucial. $R_K$ can often be estimated from sensor specifications or data. $Q_K$ is usually more of a tuning parameter, representing unmodeled dynamics and disturbances.

### 5.4 Estimators for Nonlinear Systems: EKF, UKF, and MHE

For nonlinear systems, such as most **bioreactors**, linear observers like the Luenberger observer or Kalman Filter are not directly applicable. Extensions are needed:

1.  **Extended Kalman Filter (EKF):**
    *   Applies the Kalman Filter logic to nonlinear systems by linearizing the nonlinear model around the current state estimate at each time step.
    *   Nonlinear model: $x_{k+1} = f(x_k, u_k) + G w_k$, $y_{m,k} = h(x_k, u_k) + v_k$.
    *   The matrices $A, C$ (and $G$ for noise) in the KF equations are replaced by Jacobians:
        $A_k = \frac{\partial f}{\partial x} \Big|_{\hat{x}_{k|k}, u_k}$, $C_k = \frac{\partial h}{\partial x} \Big|_{\hat{x}_{k+1|k}, u_{k+1}}$
    *   **Pros:** Conceptually straightforward extension of KF.
    *   **Cons:** Linearization can introduce significant errors if nonlinearities are strong or estimates are poor. Jacobians can be complex to derive and compute. Can diverge if not carefully tuned.

2.  **Unscented Kalman Filter (UKF):**
    *   Avoids direct linearization by using a deterministic sampling technique called the unscented transform.
    *   A set of "sigma points" are chosen around the current state estimate, propagated through the *true nonlinear model*, and then used to reconstruct the mean and covariance of the transformed distribution.
    *   **Pros:** Generally more accurate than EKF for highly nonlinear systems, no Jacobians needed.
    *   **Cons:** Computationally more intensive than EKF due to propagation of multiple sigma points. Still relies on Gaussian noise assumptions.

3.  **Moving Horizon Estimation (MHE):**
    *   The "MPC of estimation." MHE formulates state estimation as an optimization problem over a moving window of past measurements.
    *   Objective function to minimize:
        $J_{MHE} = \sum_{j=k-N_M}^{k-1} ||y_{m,j} - h(\hat{x}_j, u_j)||^2_{R_K^{-1}} + \sum_{j=k-N_M}^{k-1} ||w_j||^2_{Q_K^{-1}} + ||\hat{x}_{k-N_M} - \bar{x}_{k-N_M}||^2_{P_0^{-1}}$
        (The last term is an arrival cost penalizing deviation from a prior estimate at the start of the horizon $N_M$).
    *   Solved subject to the nonlinear model equations $x_{j+1} = f(x_j, u_j) + w_j$ and potentially constraints on states or disturbances.
    *   **Pros:** Can explicitly handle nonlinear models and constraints on states/parameters. Can provide smoothed estimates over a window. Natural fit with NMPC.
    *   **Cons:** Computationally most expensive due to repeated NLP solution. Requires careful tuning of arrival cost and horizon length.
    *   *Bioreactor Link:* MHE is well-suited for bioreactors due to nonlinearities, constraints (e.g., concentrations must be positive), and the ability to incorporate infrequent offline measurements naturally into its windowed optimization.

*Feynman Insight:* State estimation is like a detective trying to reconstruct a crime scene (the true state) using incomplete clues (noisy measurements) and a general understanding of how things work (the model). The detective constantly updates their theory as new clues arrive. Different estimators are like detectives with different methods and levels of sophistication.

### 5.5 Disturbance Modeling and Rejection: Dealing with the Unknown

Even with a good model and state estimator, unmeasured disturbances and inherent model-plant mismatch will always exist. MPC needs a mechanism to reject these disturbances and achieve its desired performance (e.g., offset-free tracking of setpoints).

1.  **Types of Disturbances:**
    *   **Input Disturbances:** Affect the inputs to the plant (e.g., variation in feed concentration $S_{feed}$ in a bioreactor).
    *   **Output Disturbances:** Affect the measured outputs directly (e.g., sensor drift).
    *   **State/Process Disturbances:** Affect the internal dynamics of the system (e.g., unexpected change in cell metabolism, $w_k$ in the KF model).

2.  **Integrating Disturbances into the Prediction Model:**
    A common approach is to assume a model for the disturbance and estimate its current value. For example, an output disturbance $d_k$ can be modeled:
    $y_k = C x_k + D u_k + d_k$
    If we assume the disturbance is a constant or slowly varying (a step disturbance), we can augment the state vector:
    $x_{aug,k} = \begin{bmatrix} x_k \\ d_k \end{bmatrix}$
    The disturbance model would be $d_{k+1} = d_k (+ \text{noise})$.
    The augmented system is then:
    $x_{aug,k+1} = \begin{bmatrix} A & 0 \\ 0 & I \end{bmatrix} x_{aug,k} + \begin{bmatrix} B \\ 0 \end{bmatrix} u_k + \text{noise}$
    $y_k = \begin{bmatrix} C & I \end{bmatrix} x_{aug,k} + D u_k + \text{noise}$
    The state estimator (e.g., Kalman Filter) now also estimates $d_k$. The MPC prediction then includes the effect of this estimated disturbance: $\hat{y}_{k+j|k} = C \hat{x}_{k+j|k} + D u_{k+j|k} + \hat{d}_{k|k}$.

3.  **Integral Action for Offset-Free Tracking:**
    To ensure that the controlled outputs reach their setpoints $r$ in the presence of sustained disturbances or model errors (i.e., achieve zero steady-state error or offset), integral action is often incorporated into MPC.
    *   **Error Augmentation:** Augment the state with the integral of the tracking error $e_k = y_k - r_k$. This is similar to the PI part of a PID controller.
    *   **Disturbance Observer:** The method described above (estimating $d_k$) effectively provides integral action if the estimated disturbance is used to correct the predictions. The MPC will adjust $u_k$ to counteract the effect of $\hat{d}_{k|k}$ to drive $\hat{y}_{k+j|k}$ towards $r_{k+j}$.
    *   *Feynman Insight:* Integral action is like the controller "learning" the constant bias or offset needed to counteract an unknown persistent force. If there's a steady headwind (disturbance), the car's cruise control (MPC with integral action) will learn to apply a bit more throttle consistently to maintain speed.

The choice of disturbance model (constant, ramp, ARIMA, etc.) depends on the expected nature of the disturbances. A well-designed disturbance model and estimator combination is key for robust MPC performance.

### 5.6 Practical Considerations for Bioreactors

*   **Infrequent and Delayed Measurements:** Offline assays for $X_v, S, P$ are a major challenge. MHE is particularly good here as it can naturally incorporate measurements with different sampling rates and delays within its optimization window. "Soft sensors" (empirical models correlating online data like OUR, CER, capacitance to offline states) can also provide intermediate estimates.
*   **Model Adaptation:** Biological systems are notorious for time-varying parameters (e.g., cells adapt, evolve, or enter different metabolic phases). Combining state estimation with online parameter estimation (e.g., Dual EKF, Joint MHE) can be beneficial, though it increases complexity and identifiability challenges.
*   **Sensor Fault Detection:** The innovation term $(y_m - \hat{y})$ in estimators can be monitored. Large, persistent innovations might indicate sensor failure or significant process upset, triggering alarms or alternative control strategies.

**This chapter has highlighted that MPC does not operate in a vacuum. It requires robust state estimation to understand its current position and effective disturbance modeling to counteract unmeasured forces. These components are essential for translating the theoretical elegance of MPC into practical, high-performing control systems. With these in place, we can now more confidently explore the intricacies of the MPC optimization engine and stability, starting with the computational aspects in the next chapter.**

---