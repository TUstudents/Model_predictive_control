## Chapter 9 (New): Advanced Disturbance Modeling and Robust Rejection in MPC

*"The art of life isn't controlling the wind, but harnessing it." - Anonymous*

While Chapter 5 introduced basic disturbance estimation for achieving offset-free tracking, real-world disturbances are often more complex than simple steps or white noise. They can be correlated, periodic, or have specific spectral characteristics. The ability of an MPC controller to effectively "harness" these disturbances – by accurately modeling them, predicting their impact, and generating appropriate counter-actions – is crucial for achieving superior performance and robustness beyond simple setpoint regulation. This chapter delves into advanced techniques for disturbance modeling, estimation, and rejection within the MPC framework.

### 9.1 Limitations of Simple Disturbance Models

The common approach of assuming an integrated white noise or step disturbance model (as in Chapter 5.5) is effective for rejecting constant or slowly varying unmeasured load disturbances and achieving offset-free control. However, it has limitations:

1.  **Correlated Disturbances:** Many industrial disturbances are not white noise but are autocorrelated (e.g., a slowly drifting raw material quality, ambient temperature variations with daily cycles). A simple step disturbance model might react sluggishly or sub-optimally to these.
2.  **Periodic Disturbances:** Some processes experience periodic disturbances (e.g., due to rotating machinery, daily/weekly operational cycles, batch additions in a semi-continuous process). A standard disturbance model won't specifically target these frequencies.
3.  **Known Measurable Disturbances (Feedforward):** Sometimes, disturbances are measurable (e.g., feed flow rate variations to a downstream unit, ambient temperature). While MPC can inherently handle these as measured inputs if they are part of the model, specific feedforward structures can enhance rejection.
4.  **Disturbance Structure:** The assumption that disturbances enter only at the output or as a simple input load might not capture how they truly affect the process states.

Ignoring these characteristics can lead to MPC performance that is less effective at disturbance rejection than it could be.

### 9.2 Modeling Stochastic Disturbances with Time Series Models

To capture more complex disturbance characteristics, time series models can be integrated into the process model used for MPC prediction and state estimation.

1.  **ARMA/ARIMA Models for Disturbances:**
    *   Autoregressive Moving Average (ARMA) models can represent stationary stochastic processes with autocorrelation.
        $d_k = \sum_{i=1}^{p_d} \phi_i d_{k-i} + \sum_{j=1}^{q_d} \theta_j e_{k-j} + e_k$
        where $e_k$ is white noise, and $\phi_i, \theta_j$ are AR and MA parameters.
    *   Autoregressive Integrated Moving Average (ARIMA) models extend ARMA to non-stationary disturbances by incorporating differencing. An ARIMA(p,d,q) model becomes an ARMA(p,q) model after differencing the disturbance $d$ times.
    *   **Integration with State-Space:** An ARIMA disturbance model can be written in state-space form and augmented to the main process state-space model:
        Original: $x_{k+1} = Ax_k + Bu_k + G_w w_k$
                  $y_k = Cx_k + Du_k + v_k$
        Disturbance: $d_{k+1} = A_d d_k + B_d e_k$
                     $y_{dist,k} = C_d d_k$
        Augmented: $\begin{bmatrix} x_{k+1} \\ d_{k+1} \end{bmatrix} = \begin{bmatrix} A & G_d C_d \\ 0 & A_d \end{bmatrix} \begin{bmatrix} x_k \\ d_k \end{bmatrix} + \begin{bmatrix} B \\ 0 \end{bmatrix} u_k + \begin{bmatrix} G_w & 0 \\ 0 & B_d \end{bmatrix} \begin{bmatrix} w_k \\ e_k \end{bmatrix}$
                     $y_k = \begin{bmatrix} C & C_d \end{bmatrix} \begin{bmatrix} x_k \\ d_k \end{bmatrix} + Du_k + v_k$
        (Here, $G_d$ maps the disturbance state $d_k$ to how it affects $x_k$, or it could directly affect $y_k$).
    *   The Kalman Filter (or its nonlinear counterparts) can then estimate both the process states $x_k$ and the disturbance states $d_k$. The MPC prediction will use these estimated disturbance states to forecast the future impact of the correlated disturbance.

2.  **Identification of Disturbance Models:**
    *   The parameters of ARMA/ARIMA models for disturbances can be identified from historical process data by analyzing the residuals (prediction errors) of a nominal process model.
    *   Techniques like Box-Jenkins methodology can be used.

*Feynman Insight:* Modeling disturbances with ARIMA is like recognizing that the "wind" isn't just random gusts (white noise) but has patterns – maybe it's stronger in the morning and calms in the evening, or if it's windy now, it's likely to be windy for the next little while. MPC with this knowledge can anticipate these patterns rather than just reacting to them moment by moment.

### 9.3 Handling Periodic Disturbances

If known periodic disturbances are present, specialized techniques can enhance rejection:

1.  **Internal Model Principle (IMP) in MPC:**
    *   The IMP states that for perfect asymptotic rejection of a disturbance (or tracking of a reference), the closed-loop system must contain a model of the disturbance (or reference) dynamics in its feedback loop.
    *   For a sinusoidal disturbance at frequency $\omega_0$, this means the controller should effectively have poles at $e^{\pm j\omega_0 T_s}$.
    *   This can be achieved in MPC by:
        *   **Augmenting the state with an oscillator model:** Add states representing a sinusoid at the disturbance frequency $\omega_0$ to the disturbance model $d_k$. The Kalman filter then estimates the amplitude and phase of this periodic disturbance.
            $A_d = \begin{bmatrix} \cos(\omega_0 T_s) & \sin(\omega_0 T_s) \\ -\sin(\omega_0 T_s) & \cos(\omega_0 T_s) \end{bmatrix}$
        *   The MPC prediction will then include this estimated periodic component, allowing the controller to generate counteracting control signals.

2.  **Repetitive Control Integrated with MPC:**
    *   Repetitive control is specifically designed for rejecting periodic disturbances with a known period $N_{rep}$ (in samples).
    *   A repetitive controller typically includes a delay line and a positive feedback loop that reinforces control actions from previous periods.
    *   This can be combined with MPC, where the MPC handles non-periodic disturbances and setpoint changes, while the repetitive component specifically targets the known periodic disturbance.

*Bioreactor Link:* Daily temperature cycles affecting cooling water, or periodic sampling routines that momentarily disturb the culture, could be candidates for these techniques if their impact is significant.

### 9.4 Explicit Feedforward Control for Measured Disturbances

When a disturbance $v_d(k)$ is measurable *before* it affects the process outputs, this information can be used proactively via feedforward control.

1.  **Standard MPC Handling:** If the measured disturbance $v_d(k)$ is included as a known input in the state-space model used by MPC:
    $x_{k+1} = Ax_k + Bu_k + B_d v_{d,k}$
    The MPC will naturally account for its predicted effect.

2.  **Enhanced Feedforward Design within MPC:**
    *   Sometimes, a more explicit feedforward term can be designed to cancel the disturbance effect perfectly (if possible) for the nominal model.
    *   The MPC then acts on the residual system or provides corrective trim.
    *   This often involves designing a feedforward gain or filter $G_{ff}(z)$ such that $P(z)G_{ff}(z) + P_d(z) \approx 0$, where $P(z)$ is the process transfer function from $u$ to $y$, and $P_d(z)$ is from $v_d$ to $y$.
    *   The MPC optimization could then determine $u_{fb,k}$ for $u_k = u_{ff,k} + u_{fb,k}$.

*Bioreactor Link:* If the concentration of a key nutrient in the feed medium ($S_{feed}$) varies but is measurable online before the feed is added, this $S_{feed}(t)$ can be used as a measured disturbance input to the MPC model, allowing it to adjust feed rates proactively.

### 9.5 Input Disturbance Estimation and Rejection

Sometimes disturbances act primarily at the process input, effectively changing the $u_k$ that the plant sees compared to what the controller commands.
$x_{k+1} = A x_k + B (u_k + d_{in,k})$
$y_k = C x_k + D (u_k + d_{in,k})$

*   **Estimation:** An input disturbance $d_{in,k}$ can be estimated by augmenting the state (similar to output disturbance estimation) or by using specific observer structures.
*   **Rejection:** Once estimated, the MPC can compensate by adjusting its commanded $u_k$ to counteract $\hat{d}_{in,k}$.

This is particularly relevant if there's actuator bias, unmodeled actuator dynamics, or if the "true" input to a critical part of the process is affected by upstream variations.

### 9.6 Robustness of Disturbance Rejection Mechanisms

*   The effectiveness of advanced disturbance rejection relies on the accuracy of the disturbance model itself. If the disturbance model is poor (e.g., wrong frequency for a periodic disturbance, incorrect ARMA parameters), performance can degrade.
*   **Sensitivity to Disturbance Model Mismatch:** It's important to analyze how sensitive the MPC performance is to errors in the assumed disturbance characteristics.
*   **Adaptive Disturbance Models:** For slowly changing disturbance characteristics, parameters of the disturbance model (e.g., ARMA coefficients, dominant frequency) could be adapted online. This leads to adaptive MPC strategies.

### 9.7 Nonlinear Disturbance Modeling and Estimation

For NMPC applications where disturbances interact nonlinearly with the system or the disturbance generation mechanism itself is nonlinear:

1.  **Nonlinear Disturbance Models:** The augmented disturbance states $d_k$ might follow nonlinear dynamics: $d_{k+1} = f_d(d_k, x_k, e_k)$.
2.  **Nonlinear Estimators (EKF, UKF, MHE):** These estimators (Chapter 5) are used to estimate both the process states and the nonlinear disturbance states. MHE is particularly powerful as it can incorporate nonlinear disturbance models and constraints directly.
3.  **Example:** If a byproduct $L$ (acting as an internal disturbance) inhibits growth $\mu$ in a highly nonlinear way (e.g., $\mu = \mu_0 / (1 + (L/K_L)^2)$), and $L$ itself is produced via a nonlinear pathway, then estimating and predicting $L$ accurately using a nonlinear model is key for the NMPC to take appropriate actions.

*Feynman Insight:* Advanced disturbance modeling is like a seasoned sailor who doesn't just feel the current wind but also understands ocean currents, tidal patterns, and how different sail trims interact with complex wave patterns. This deeper understanding allows for much more sophisticated and effective navigation (control).

**By moving beyond simple disturbance assumptions and employing more sophisticated models that capture the true nature of process variations, MPC can achieve a significantly higher level of performance in terms of disturbance rejection and overall process stability. This often requires more effort in model identification and estimator design but can yield substantial benefits in challenging industrial applications where disturbances are a major limiting factor for performance.**

---