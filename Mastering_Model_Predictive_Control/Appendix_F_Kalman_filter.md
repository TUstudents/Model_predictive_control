## Appendix F: The Kalman Filter – Optimal State Estimation for Linear Systems with Noise

In the real world, process measurements are invariably corrupted by noise, and the systems themselves are subject to unmeasured disturbances or random fluctuations. For Model Predictive Control to perform effectively, it needs an accurate estimate of the system's current state, despite these imperfections. The **Kalman Filter (KF)**, developed by Rudolf E. Kálmán in the early 1960s, is a powerful and widely used recursive algorithm that provides an optimal estimate of the state of a linear dynamic system perturbed by Gaussian white noise, using measurements that are also corrupted by Gaussian white noise. This appendix revisits and expands on the Kalman Filter, highlighting its structure, optimality, and crucial role in enabling practical MPC.

### F.1 The Problem: State Estimation in the Presence of Noise

Consider a discrete-time linear time-invariant (LTI) system described by:

**Process Model (State Equation):**
$x_{k+1} = A x_k + B u_k + G w_k \quad \quad (F.1)$

**Measurement Model (Output Equation):**
$y_{m,k} = C x_k + D u_k + v_k \quad \quad (F.2)$

Where:
*   $x_k \in \mathbb{R}^n$: State vector at time $k$.
*   $u_k \in \mathbb{R}^m$: Control input vector at time $k$ (known).
*   $y_{m,k} \in \mathbb{R}^p$: Measured output vector at time $k$.
*   $A, B, C, D, G$: System matrices of appropriate dimensions.
*   $w_k \in \mathbb{R}^q$: **Process noise** vector at time $k$. This represents unmodeled dynamics, random disturbances affecting the state evolution, or uncertainties in the system model. It is assumed to be a zero-mean, Gaussian white noise sequence with covariance matrix $Q_K$:
    $E[w_k] = 0, \quad E[w_k w_j^T] = Q_K \delta_{kj}$ (where $\delta_{kj}$ is the Kronecker delta).
*   $v_k \in \mathbb{R}^p$: **Measurement noise** vector at time $k$. This represents sensor inaccuracies or random errors in the measurement process. It is assumed to be a zero-mean, Gaussian white noise sequence with covariance matrix $R_K$:
    $E[v_k] = 0, \quad E[v_k v_j^T] = R_K \delta_{kj}$.
*   It is also assumed that $w_k$ and $v_k$ are uncorrelated with each other and with the initial state $x_0$.
    $E[w_k v_j^T] = 0$ for all $k,j$.

The goal of the Kalman Filter is to find an estimate of the true state $x_k$, denoted $\hat{x}_{k|k}$ (the estimate of $x_k$ given measurements up to time $k$), that is "optimal" in some sense.

### F.2 The Kalman Filter Algorithm: A Two-Step Recursive Process

The Kalman Filter operates recursively in two steps at each time interval:

1.  **Time Update (Prediction Step):**
    Given the state estimate $\hat{x}_{k-1|k-1}$ and its error covariance $P_{k-1|k-1}$ from the previous time step, this step predicts the state and error covariance for the current time $k$, *before* incorporating the current measurement $y_{m,k}$. These are called the *a priori* estimates.
    *   **Project the state estimate forward:**
        $\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_{k-1} \quad \quad (F.3)$
        (This is simply the system model without the noise term, using the best previous estimate).
    *   **Project the error covariance forward:**
        $P_{k|k-1} = A P_{k-1|k-1} A^T + G Q_K G^T \quad \quad (F.4)$
        ($P_{k|k-1} = E[(x_k - \hat{x}_{k|k-1})(x_k - \hat{x}_{k|k-1})^T]$ is the covariance of the a priori state estimation error). This equation shows how the uncertainty from the previous estimate ($A P_{k-1|k-1} A^T$) and the process noise ($G Q_K G^T$) contribute to the uncertainty in the current prediction.

2.  **Measurement Update (Correction Step):**
    Once the current measurement $y_{m,k}$ becomes available, this step incorporates that new information to refine the a priori estimates, producing the *a posteriori* state estimate $\hat{x}_{k|k}$ and its error covariance $P_{k|k}$.
    *   **Compute the Kalman Gain ($K_k$):**
        $K_k = P_{k|k-1} C^T (C P_{k|k-1} C^T + R_K)^{-1} \quad \quad (F.5)$
        The Kalman gain $K_k \in \mathbb{R}^{n \times p}$ determines how much weight is given to the measurement residual (the difference between the actual measurement and the predicted measurement). It essentially balances the confidence in the prediction (from $P_{k|k-1}$) versus the confidence in the measurement (from $R_K$).
        If $R_K$ is small (measurements are accurate), $K_k$ will be larger, giving more weight to the measurement.
        If $P_{k|k-1}$ is small (predictions are accurate), $K_k$ will be smaller, giving more weight to the prediction.
    *   **Update the state estimate with measurement $y_{m,k}$:**
        $\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (y_{m,k} - (C \hat{x}_{k|k-1} + D u_k)) \quad \quad (F.6)$
        The term $(y_{m,k} - (C \hat{x}_{k|k-1} + D u_k))$ is called the **innovation** or measurement residual. It's the new information brought by the measurement.
    *   **Update the error covariance:**
        $P_{k|k} = (I - K_k C) P_{k|k-1} \quad \quad (F.7)$
        (This can also be written in other forms, e.g., $P_{k|k} = (I - K_k C) P_{k|k-1} (I - K_k C)^T + K_k R_K K_k^T$, which is more numerically stable if $P_{k|k-1}$ is not perfectly symmetric due to round-off). This equation shows how incorporating the measurement reduces the uncertainty in the state estimate.

**Initialization:**
To start the recursion, initial values for the state estimate $\hat{x}_{0|-1}$ (or $\hat{x}_{0|0}$) and its error covariance $P_{0|-1}$ (or $P_{0|0}$) are needed.
*   $\hat{x}_{0|-1}$ can be an educated guess of the initial state.
*   $P_{0|-1}$ reflects the uncertainty in this initial guess. If very uncertain, $P_{0|-1}$ can be set to a diagonal matrix with large values. The filter often converges to good estimates even from poor initial guesses if the system is observable.

**(Figure F.1: Block diagram illustrating the two-step predict-correct cycle of the Kalman Filter.)**

### F.3 Optimality and Properties of the Kalman Filter

*   **Optimality:** Under the assumptions of linearity and Gaussian noise, the Kalman Filter is optimal in several senses:
    *   It is the **Minimum Mean Square Error (MMSE)** estimator: $E[||x_k - \hat{x}_{k|k}||^2]$ is minimized.
    *   It is the **Maximum A Posteriori (MAP)** estimator: It finds the state estimate that maximizes the posterior probability $p(x_k | y_{m,1}, \dots, y_{m,k})$.
    *   It is the Best Linear Unbiased Estimator (BLUE) even if the noise is not Gaussian, provided it's zero-mean and has covariances $Q_K, R_K$.
*   **Unbiased:** $E[x_k - \hat{x}_{k|k}] = 0$, meaning the estimate is correct on average.
*   **Covariance Matrix:** $P_{k|k}$ is the actual covariance of the estimation error $x_k - \hat{x}_{k|k}$. This provides a measure of the filter's accuracy.
*   **Steady-State Behavior:** For LTI systems, if $(A, G\sqrt{Q_K})$ is stabilizable and $(A,C)$ is detectable, the error covariance matrix $P_{k|k}$ and the Kalman gain $K_k$ often converge to steady-state values $P_\infty$ and $K_\infty$ as $k \rightarrow \infty$. In such cases, a constant gain Kalman filter (using $K_\infty$) can be used, simplifying implementation. $P_\infty$ is the solution to a Discrete Algebraic Riccati Equation (DARE).

### F.4 Tuning the Kalman Filter: The Role of $Q_K$ and $R_K$

The performance of the Kalman Filter heavily depends on the correct specification of the noise covariance matrices $Q_K$ and $R_K$.

*   **Measurement Noise Covariance ($R_K$):**
    *   This matrix reflects the accuracy of the sensors. The diagonal elements are the variances of the noise on each sensor. Off-diagonal elements are covariances if sensor noises are correlated (often assumed diagonal if sensors are independent).
    *   $R_K$ can often be estimated from sensor specifications or by analyzing sensor data when the system is in a known state or at steady state.
    *   Smaller $R_K$ values indicate more trust in the measurements.

*   **Process Noise Covariance ($Q_K$):**
    *   This matrix is often more difficult to determine accurately as it represents unmodeled dynamics, parameter uncertainties, and actual process disturbances. $G$ maps these abstract noises to states.
    *   $Q_K$ is frequently used as a "tuning knob" for the filter.
    *   Larger $Q_K$ values indicate less trust in the process model ($A, B$) and imply that the true state can deviate more significantly from the model's prediction. This will lead to a larger Kalman gain $K_k$, making the filter more responsive to measurements.
    *   Smaller $Q_K$ values imply high confidence in the model, leading to smoother state estimates that rely less on noisy measurements.
    *   If $Q_K$ is chosen too small, the filter might become overconfident in a potentially inaccurate model and diverge from the true state (especially if there are unmodeled disturbances).
    *   If $Q_K$ is too large, the estimates might become too noisy, essentially tracking the measurements too closely.

*Feynman Insight:* Tuning $Q_K$ and $R_K$ is like balancing your trust between what your instruments (sensors, giving $y_m$) are telling you and what your theory/map (model $A,B$) predicts. If your instruments are very precise ($R_K$ small), you listen to them more. If your map is known to be excellent and the terrain smooth ($Q_K$ small), you trust your map more, even if an instrument reading seems a bit off. The Kalman gain is the intelligent arbiter that finds the optimal balance based on these specified confidences.

### F.5 Connection to MPC

In an MPC framework (especially LMPC):
1.  At each time step $k$, the Kalman Filter (or its nonlinear extensions EKF/UKF for NMPC) takes the previous control input $u_{k-1}$ and the current measurement $y_{m,k}$ to produce the current state estimate $\hat{x}_{k|k}$.
2.  This $\hat{x}_{k|k}$ is then used as the initial state $x_k$ for the MPC's prediction and optimization phase to calculate $u_k^*$.
3.  The combination of MPC and a Kalman filter is often referred to as an LQG (Linear Quadratic Gaussian) controller when the objective is quadratic and noises are Gaussian, though MPC has explicit constraint handling which LQG does not.

The separation principle in classical LQG control states that the optimal controller design (LQR) and the optimal state estimator design (Kalman Filter) can be done separately. While this strict separation doesn't always hold perfectly for constrained MPC, the practice of designing the estimator and controller somewhat independently is common and often effective.

### F.6 Limitations and Extensions

*   **Linearity Assumption:** The standard KF assumes linear system dynamics and linear measurement models.
*   **Gaussian Noise Assumption:** Optimality (in MMSE/MAP sense) relies on Gaussian noise. However, the KF is still the best *linear* unbiased estimator even for non-Gaussian noise.
*   **Known Noise Statistics:** Assumes $Q_K$ and $R_K$ are known. In practice, adaptive Kalman filters exist that try to estimate these online.
*   **Extensions for Nonlinearity:**
    *   **Extended Kalman Filter (EKF):** Linearizes the nonlinear model at each step (Chapter 5, Appendix A if ANN model).
    *   **Unscented Kalman Filter (UKF):** Uses sigma points to propagate uncertainty through nonlinearities (Chapter 5).
    *   **Particle Filters:** For highly non-Gaussian systems, use a set of weighted samples (particles) to represent the posterior distribution. Computationally intensive.
    *   **Moving Horizon Estimation (MHE):** An optimization-based approach for nonlinear systems (Chapter 5).

**The Kalman Filter is a cornerstone of modern control and signal processing. Its recursive structure, optimality properties, and ability to provide a measure of estimation uncertainty make it an indispensable tool for obtaining reliable state estimates from noisy data, thereby enabling the successful practical application of Model Predictive Control and many other advanced control strategies.**

---