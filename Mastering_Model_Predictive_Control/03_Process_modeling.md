## Chapter 3: Process Modeling for Control: Principles and Linear Models

*"The purpose of computing is insight, not numbers." - Richard Hamming*

In Model Predictive Control, the "Model" is not just part of the name; it's the very heart of the strategy. As Hamming's quote suggests, the numbers our models produce are only valuable if they grant us the insight to predict and control our processes effectively. This chapter delves into the principles of process modeling for control, with an initial focus on linear models, which form the bedrock of LMPC. We'll explore how these models are obtained, validated, and what their limitations are, always remembering that our ultimate goal is effective control.

### 3.1 The Role of Models in Control: MPC's "Crystal Ball"

As established in Chapter 1, MPC relies on a dynamic model to predict the future behavior of the system. This model serves as a surrogate for the real process within the controller's optimization algorithm.

**Why is an accurate model so crucial?**

1.  **Prediction Accuracy:** The quality of the MPC's decisions directly depends on how well its internal model can forecast the process response to control inputs. A poor model leads to poor predictions, and consequently, suboptimal or even unstable control actions.
2.  **Constraint Handling:** MPC uses the model to predict if future inputs or outputs will violate constraints. If the model inaccurately predicts these trajectories, the controller might either act too conservatively (missing performance opportunities) or too aggressively (violating actual process limits).
3.  **Understanding System Dynamics:** The process of developing a model often forces engineers to gain a deeper understanding of the underlying system dynamics, interactions, and important variables. This insight itself is valuable.
4.  **Simulation and Tuning:** The model is used extensively offline for simulating control strategies, tuning MPC parameters ($\mathbf{Q}, \mathbf{R}, N_p, N_c$), and testing controller robustness before deployment on the actual plant.

However, it's vital to embrace George Box's famous aphorism:

*"All models are wrong, but some are useful."*

No model will ever be a perfect representation of reality, especially for complex industrial or biological systems. Models are simplifications. The key is to develop a model that is "useful" for the control task at hand – one that captures the dominant dynamics relevant to the control objectives and constraints within the operating region of interest, and whose complexity is manageable for online computation.

*Feynman Insight:* Think of a weather forecast. It's based on complex atmospheric models. No forecast is ever 100% accurate, but a good forecast (a useful model) allows you to decide whether to carry an umbrella (make a good control decision). A consistently wrong forecast (a bad model) is useless or even detrimental.

### 3.2 Modeling Paradigms: A Spectrum of Approaches

Process models can be broadly categorized, often falling along a spectrum:

1.  **First-Principles (Mechanistic or White-Box) Models:**
    *   Based on fundamental laws of physics, chemistry, and biology (e.g., conservation of mass, energy, momentum; reaction kinetics; transport phenomena).
    *   Typically result in sets of ordinary differential equations (ODEs) or partial differential equations (PDEs), and algebraic equations (DAEs).
    *   **Pros:** Offer deep process insight, can extrapolate better beyond the data used for parameter fitting (if the underlying principles are correct), parameters often have physical meaning.
    *   **Cons:** Can be very complex to develop, may require knowledge of many physical/chemical parameters that are hard to determine accurately, might be too computationally intensive for real-time MPC if very detailed.
    *   *Example:* A detailed kinetic model of a chemical reactor, a Navier-Stokes model of fluid flow.

2.  **Empirical (Data-Driven or Black-Box) Models:**
    *   Derived primarily from observed input-output data from the process, with minimal or no explicit incorporation of underlying physical laws.
    *   The model structure is chosen (e.g., linear state-space, transfer function, neural network), and parameters are fitted to match the data.
    *   **Pros:** Can be developed relatively quickly if sufficient data is available, can capture complex behaviors without needing full mechanistic understanding, often computationally efficient if linear.
    *   **Cons:** Typically interpolate well but extrapolate poorly, offer less process insight, performance heavily depends on the quality and richness of the identification data, may not capture fundamental process changes.
    *   *Example:* A step-response model, an ARX model, a neural network trained on plant data.

3.  **Hybrid (Grey-Box) Models:**
    *   Combine elements of both first-principles and empirical modeling.
    *   For example, a model might have a known mechanistic structure, but some parameters or sub-models within it are determined empirically from data.
    *   **Pros:** Can leverage existing process knowledge while using data to fill gaps or simplify complex parts, often a good balance of insight and accuracy.
    *   **Cons:** Can be challenging to decide how to best combine the two approaches.
    *   *Example:* A bioreactor model where the overall mass balances are first-principles, but the specific growth rate kinetics are represented by an empirical function fitted to experimental data.

For LMPC, we typically linearize a first-principles model around an operating point or identify a linear empirical model directly. Chapter 4 will focus on nonlinear modeling, more suited for NMPC.

**Continuous-Time vs. Discrete-Time Models:**
While many physical processes evolve continuously in time (described by ODEs/PDEs), MPC is implemented on digital computers. Thus, for LMPC, we usually work with:
*   Discrete-time models obtained by direct identification from sampled data.
*   Discrete-time models obtained by discretizing a continuous-time model (as shown in Chapter 2, equations for $A$ and $B$ from $A_c, B_c$).

### 3.3 Linear System Identification Techniques for LMPC

System identification is the art and science of building mathematical models of dynamical systems based on observed input-output data. For LMPC, we are interested in identifying discrete-time linear models, often in state-space form ($x_{k+1} = Ax_k + Bu_k, y_k = Cx_k + Du_k$) or input-output forms like ARX, ARMAX, etc.

**Key Steps in System Identification:**

1.  **Experiment Design:**
    *   The quality of the identification data is paramount. "Garbage in, garbage out."
    *   **Input Signal Design:** The input signal used to excite the system during the identification experiment must be "persistently exciting" enough to reveal the system's dynamics.
        *   **Pseudo-Random Binary Sequence (PRBS):** A sequence that switches between two levels (e.g., $u_{min}, u_{max}$) with pseudo-random timing. Good for exciting a broad range of frequencies.
        *   **Schroeder-Phased Signals (Multisine):** Sums of sinusoids with phases chosen to minimize the signal's crest factor (peak-to-average ratio), allowing for higher power input without saturating actuators.
        *   The input should excite frequencies relevant to the desired closed-loop bandwidth.
    *   **Sampling Time ($T_s$):** Chosen based on the system's dominant time constants (e.g., 5-10 samples per dominant rise time is a rule of thumb).
    *   **Experiment Length:** Long enough to capture slow dynamics and reduce noise effects.

2.  **Data Preprocessing:**
    *   Removing outliers.
    *   Filtering to remove noise outside the frequency band of interest.
    *   Removing means/trends (to focus on dynamic behavior).

3.. **Model Structure Selection:**
    *   Choosing the type of linear model (e.g., state-space, ARX, FIR) and its order (e.g., number of states $n$ for state-space, number of past inputs/outputs for ARX).
    *   This often involves some trial and error or using information criteria (AIC, BIC).

4.  **Parameter Estimation:**
    *   Using algorithms to find the model parameters that best fit the observed data.
    *   **Least Squares (LS):** Common for models linear in parameters (e.g., FIR, ARX).
    *   **Prediction Error Methods (PEM):** A powerful class of methods that minimize the error between the model's predictions and the measured outputs. ARMAX, Output-Error (OE), and Box-Jenkins (BJ) models use PEM. State-space models can also be identified using PEM.
    *   **Subspace Identification Methods (e.g., N4SID, MOESP, CVA):** These methods directly estimate state-space matrices ($A,B,C,D$) from input-output data without needing to specify a model order beforehand (though a choice is made post-estimation). They are particularly good for MIMO systems.
        *   *N4SID (Numerical algorithms for Subspace State Space System IDentification):* Uses projections of input-output data matrices and Singular Value Decomposition (SVD) to estimate the extended observability matrix, from which state-space matrices can be extracted.

5.  **Model Validation:**
    *   Assessing how well the identified model represents the true system. This is crucial!
    *   **Residual Analysis:** Examining the "leftovers" (the difference between model prediction and actual output). For a good model, residuals should be small, uncorrelated (like white noise), and uncorrelated with past inputs.
    *   **Cross-Validation:** Testing the model on a *different* dataset than the one used for parameter estimation. This is the gold standard for checking generalization capability.
    *   **Simulation:** Comparing the model's simulated output to actual plant output for a given input sequence.
    *   **Frequency Response Comparison:** Comparing Bode plots if a frequency response of the plant is available.
    *   **Parsimony:** Favoring simpler models that explain the data sufficiently well (Occam's Razor).

**Common Linear Model Structures:**

*   **Finite Impulse Response (FIR):** $y_k = \sum_{i=0}^{N_b} b_i u_{k-i} + e_k$. Simple, always stable, but can require many parameters for systems with slow dynamics.
*   **ARX (AutoRegressive with eXternal input):** $A(q) y_k = B(q) u_k + e_k$, where $A(q), B(q)$ are polynomials in the backshift operator $q^{-1}$ (i.e., $q^{-1}y_k = y_{k-1}$). Solved efficiently using least squares. Noise $e_k$ is assumed white.
*   **ARMAX (AutoRegressive Moving Average with eXternal input):** $A(q) y_k = B(q) u_k + C(q) e_k$. More flexible noise modeling than ARX, but requires iterative PEM.
*   **Output-Error (OE):** $y_k = \frac{B(q)}{F(q)} u_k + e_k$. Models the process dynamics separately from the noise dynamics.
*   **State-Space Models:** As in (2.1). These are often preferred for MPC due to their explicit representation of internal states, which can be useful for estimation and prediction. Subspace methods directly identify these.

### 3.4 Input Signal Design for Identification: "Asking the Right Questions"

The quality of the input signal used during an identification experiment directly impacts the quality of the resulting model.

*   **Persistent Excitation:** The input must be "rich" enough in frequencies to excite all relevant modes of the system. A constant input, for example, provides no dynamic information.
*   **Frequency Content:** The input signal should have significant energy in the frequency range where accurate control is desired.
*   **Amplitude:** The input amplitude should be large enough to provide a good signal-to-noise ratio (SNR) but small enough to keep the system within its (approximately) linear operating region and avoid violating safety constraints.
*   **PRBS (Pseudo-Random Binary Sequence):**
    *   Switches between two levels ($u_1, u_2$).
    *   Properties determined by its length ($N=2^L-1$) and clock period ($T_{clock}$).
    *   Power spectrum is approximately flat up to a certain frequency, then rolls off.
    *   Easy to implement.
    **(Figure 3.1: Example of a PRBS signal and its power spectrum.)**
*   **Multisine Signals (Schroeder-Phased):**
    *   $u(t) = \sum_{k=1}^{F} A_k \sin(2\pi f_k t + \phi_k)$
    *   Phases $\phi_k$ are chosen to minimize the crest factor, allowing more energy input for given amplitude constraints.
    *   Can tailor the frequency content more precisely.

### 3.5 Model Validation and Selection: "Trust, but Verify"

Once parameters for a candidate model structure are estimated, rigorous validation is non-negotiable.

*   **One-Step-Ahead Prediction vs. Simulation:**
    *   A model might give good one-step-ahead predictions (where $\hat{y}_{k|k-1}$ uses actual $y_{k-1}, y_{k-2}, \dots$) but perform poorly in simulation (where $\hat{y}_{k|k-1}$ uses previously *predicted* outputs $\hat{y}_{k-1}, \hat{y}_{k-2}, \dots$). For MPC, good simulation performance (multi-step prediction) is vital.
*   **Information Criteria (for model order selection):**
    *   **Akaike Information Criterion (AIC):** Balances goodness-of-fit against model complexity.
    *   **Bayesian Information Criterion (BIC):** Similar to AIC but penalizes complexity more heavily.
    *   Lower AIC/BIC values are generally preferred.
*   **Practical Considerations:**
    *   Does the model gain make physical sense?
    *   Are poles/zeros in reasonable locations (for stability and expected response)?
    *   Does the model respond correctly to step inputs?

### 3.6 Model Uncertainty: Acknowledging Imperfection

As "all models are wrong," there will always be some **model uncertainty** or **model mismatch** between the identified linear model and the true process. This uncertainty can arise from:

*   **Unmodeled Dynamics:** Higher-order dynamics, small time delays, or nonlinearities not captured by the linear model structure.
*   **Noise:** Measurement noise and process disturbances affecting the identification data.
*   **Parameter Estimation Errors:** Finite data length and noise lead to uncertainty in estimated parameters.
*   **Time-Varying Nature of the Process:** The process itself might change over time (e.g., catalyst deactivation, fouling, changes in raw material for a **bioreactor**).

Understanding and (ideally) quantifying this uncertainty is crucial for designing robust MPC controllers (Chapter 7) that can perform reliably despite these imperfections. For LMPC, a common approach is to assume the true plant lies within a "band" of uncertainty around the nominal identified model.

### 3.7 Practical Example: Identifying a Linear Model for a Thermal Process

Consider a simple stirred tank heater. The input ($u$) is the power to the heater, and the output ($y$) is the liquid temperature.
1.  **Experiment Design:** Apply a PRBS input for heater power, varying between, say, 20% and 80% of max power, with a clock period chosen based on the expected thermal time constant. Record temperature data.
2.  **Data Preprocessing:** Remove any initial transient until the tank is roughly at an operating temperature. Detrend the data if there's a slow drift.
3.  **Model Structure & Estimation:**
    *   Try fitting a first-order plus dead-time (FOPDT) model: $K e^{-\theta s} / (\tau s + 1)$. Discretize it.
    *   Alternatively, try identifying an ARX model (e.g., `arx(data, [na nb nk])` in MATLAB's System Identification Toolbox, where `na` is order of A poly, `nb` of B poly, `nk` is delay).
    *   Or use N4SID to get a state-space model directly.
4.  **Validation:**
    *   Compare the simulated output of the identified model(s) against a validation dataset (different from the training data).
    *   Check residuals for whiteness and correlation.
    *   Select the simplest model that gives adequate prediction performance.

This identified linear model, if validated properly, could then be used to design an LMPC to regulate the tank temperature, respecting constraints on heater power.

**In the next chapter, we will turn our attention to the more challenging but often necessary task of nonlinear modeling, with a particular emphasis on developing models for bioreactors, which are prime candidates for Nonlinear MPC.** The principles of careful experimentation and rigorous validation learned here for linear systems remain equally, if not more, important in the nonlinear domain.

---