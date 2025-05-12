## Jupyter Notebook Series: "Interactive MPC: From Classical Examples to Bioreactor Control" (Revised Outline with Data-Driven Model Notebooks)

**Overall Goal:** To provide an interactive, code-driven learning experience for Model Predictive Control, allowing users to experiment with concepts, visualize results, and build up to complex applications like bioreactor control, including data-driven modeling approaches.
**Primary Language:** Python (using NumPy, SciPy, Matplotlib, CVXPY/OSQP/qpOASES for LMPC, CasADi/IPOPT for NMPC, TensorFlow/Keras or PyTorch for NNs/PINNs, GPy/GPflow for GPs).

---

**Notebook Series Outline:**

**Part 0: Introduction and Setup**

*   **Notebook 0.0: Welcome to Interactive MPC & Python Setup**
    *   Brief overview of the notebook series and its goals.
    *   Instructions for setting up the Python environment (Anaconda, Jupyter, necessary libraries like NumPy, SciPy, Matplotlib, CVXPY, CasADi, control).
    *   Basic Python/NumPy/Matplotlib tutorial or links to resources.
    *   "Hello, MPC World!" - A very simple conceptual illustration.

**Part 1: Foundations of Linear MPC (LMPC)**

*   **Notebook 1.1: Discrete-Time Linear Systems & Prediction**
    *   Defining LTI state-space models in Python.
    *   Discretizing continuous-time systems.
    *   Implementing the prediction equations:
        *   Step-by-step state and output prediction.
        *   Building the $\mathbf{F}$ and $\mathbf{\Phi}$ matrices.
    *   *Interactive Example:* Predicting the trajectory of a double integrator or a simple RC circuit given an input sequence. Visualizing free and forced responses.

*   **Notebook 1.2: LMPC Objective Function and QP Formulation**
    *   Defining quadratic objective functions (tracking, input effort, input rate).
    *   Implementing weighting matrices.
    *   Translating the LMPC problem into the standard QP form: $\frac{1}{2} \mathbf{U}^T \mathbf{H}_{QP} \mathbf{U} + \mathbf{f}_{QP}^T \mathbf{U}$.
    *   Deriving $\mathbf{H}_{QP}$ and $\mathbf{f}_{QP}$ in code.
    *   *Interactive Example:* Formulating the QP for the double integrator for a setpoint change.

*   **Notebook 1.3: Solving QPs & Basic LMPC (Unconstrained)**
    *   Introduction to QP solvers available in Python (e.g., `CVXPY` with OSQP/ECOS, or directly using `qpOASES` or `OSQP` via their Python bindings).
    *   Implementing an unconstrained LMPC loop (predict, optimize, apply first input, repeat).
    *   *Interactive Example:* Unconstrained LMPC for setpoint tracking of the double integrator. Visualizing states, inputs, and outputs. Experimenting with $\mathbf{Q}, \mathbf{R}$ weights.

*   **Notebook 1.4: LMPC with Input and Output Constraints**
    *   Formulating linear input constraints ($u_{min}, u_{max}, \Delta u_{min}, \Delta u_{max}$).
    *   Formulating linear output constraints ($y_{min}, y_{max}$).
    *   Adding these to the QP problem ($A_{QP}\mathbf{U} \le b_{QP}$).
    *   *Interactive Example:* Constrained LMPC for the double integrator (e.g., limited input force, limited velocity/position). Visualizing active constraints. Observing performance changes.

**Part 2: Modeling and State Estimation**

*   **Notebook 2.1: System Identification for LMPC (Simple Example)**
    *   Brief overview of system ID concepts.
    *   *Interactive Example:* Generating data from a known LTI system with noise. Using a simple method (e.g., least squares for an FIR or ARX model, or `scipy.signal.ss2tf` and back for simple cases) to identify a model. Comparing identified model predictions to true system.
    *   (Optional advanced section: Subspace ID using `python-control` or other libraries).

*   **Notebook 2.2: The Kalman Filter for LMPC**
    *   Implementing the discrete-time Kalman Filter equations (predict and update steps).
    *   Tuning $Q_K$ and $R_K$.
    *   Integrating the KF into the LMPC loop (output feedback MPC).
    *   *Interactive Example:* LMPC for the double integrator with noisy measurements, using a KF for state estimation. Comparing performance with/without KF, and with different $Q_K, R_K$ values.

**Part 3: Nonlinear MPC (NMPC)**

*   **Notebook 3.1: Introduction to Nonlinear Modeling & Simulation**
    *   Defining nonlinear ODE models in Python (e.g., Van der Pol oscillator, simple CSTR).
    *   Using `scipy.integrate.solve_ivp` for simulating nonlinear systems.
    *   Visualizing nonlinear behavior (phase plots, limit cycles).

*   **Notebook 3.2: NMPC Formulation and NLP Solvers**
    *   The NMPC problem: nonlinear model in prediction, nonlinear objective/constraints.
    *   Introduction to CasADi for symbolic modeling, automatic differentiation, and NLP formulation.
    *   Interfacing CasADi with an NLP solver like IPOPT.
    *   Discretization methods (briefly: single shooting, multiple shooting concepts).
    *   *Interactive Example:* Formulating an NMPC problem for the Van der Pol oscillator or a simple CSTR to track a setpoint using CasADi and IPOPT (single shooting).

*   **Notebook 3.3: Basic NMPC Implementation (with Full State Feedback)**
    *   Implementing the NMPC loop (predict using nonlinear model, optimize NLP, apply first input).
    *   *Interactive Example:* NMPC for setpoint tracking of the Van der Pol oscillator or CSTR. Visualizing performance. Experimenting with NMPC tuning parameters ($N_p, N_c, Q, R$).

*   **Notebook 3.4: NMPC with State Estimation (EKF/UKF)**
    *   Brief implementation or conceptual link to EKF/UKF for nonlinear systems (could reference external libraries or simplify).
    *   Integrating the estimator with the NMPC loop.
    *   *Interactive Example:* NMPC for the CSTR with noisy measurements, using a conceptual EKF/UKF. (Full MHE might be too complex for an early NMPC notebook but could be a later advanced one).

**Part 4: Advanced MPC Concepts (Selected Topics)**

*   **Notebook 4.1: Offset-Free Tracking & Disturbance Rejection in MPC**
    *   Implementing an augmented state with disturbance estimation (as in Chapter 5/9 of the course).
    *   *Interactive Example:* LMPC/NMPC for a system with an unmeasured step input or output disturbance. Demonstrating offset-free tracking.

*   **Notebook 4.2: Introduction to Economic MPC (EMPC)**
    *   Formulating an economic objective function.
    *   Comparing EMPC with tracking MPC.
    *   *Interactive Example:* Simple LMPC or NMPC system (e.g., CSTR) where the objective is to maximize product formation rate or minimize energy, rather than track a specific temperature setpoint.

*   **(Optional) Notebook 4.3: Introduction to Robust MPC (e.g., Tube-based concept or Scenario MPC for LTI)**
    *   Conceptual introduction to handling model uncertainty.
    *   *Interactive Example:* LMPC with parameter uncertainty, showing how nominal MPC might fail and how a simple robust approach (e.g., constraint tightening or evaluating a few scenarios) might improve things.

**Part 5: Bioreactor Case Studies**

*   **Notebook 5.1: Modeling a Fed-Batch Bioreactor**
    *   Implementing the nonlinear ODE model for a simplified fed-batch bioreactor (e.g., states: $X_v, S_{glc}, P_{prod}, V$; kinetics: Monod for growth, Luedeking-Piret for product).
    *   Simulating open-loop responses to different feeding profiles. Visualizing cell growth, substrate consumption, product formation.
    *   *Interactive Example:* Explore parameter sensitivity (e.g., how $\mu_{max}, K_S$ affect the batch trajectory).

*   **Notebook 5.2: NMPC for Fed-Batch Bioreactor - Setpoint Tracking**
    *   Defining control objectives (e.g., track a target glucose concentration profile $S_{glc,sp}(t)$ by manipulating feed rate $F(t)$).
    *   Formulating the NMPC problem using CasADi and the bioreactor model.
    *   Implementing the NMPC loop assuming full state feedback initially.
    *   *Interactive Example:* NMPC controlling glucose in the fed-batch bioreactor. Compare to a "bolus" or pre-defined feed profile. Add constraints (max feed rate, min/max glucose).

*   **Notebook 5.3: Economic NMPC for Fed-Batch Bioreactor - Maximizing Product**
    *   Defining an economic objective (e.g., maximize final product $P_{prod}(t_f)$).
    *   The NMPC/EMPC optimizes the feeding profile $F(t)$ over the entire (remaining) batch duration.
    *   *Interactive Example:* EMPC finding an optimal feeding strategy for the fed-batch bioreactor. Compare the resulting titer and profiles with the tracking NMPC and simpler strategies.

*   **(Optional Advanced) Notebook 5.4: State Estimation for Bioreactors & NMPC with Estimated States**
    *   Discussing challenges of bioreactor state estimation (infrequent offline samples).
    *   Implementing a conceptual EKF or demonstrating how MHE would be structured for the bioreactor model.
    *   Running the NMPC/EMPC from Notebook 5.2 or 5.3 using estimated states.

*   **(Optional) Notebook 5.5: Introduction to Perfusion Bioreactor Modeling & Control Concepts**
    *   Implementing a simplified perfusion bioreactor model.
    *   Discussing control objectives (maintain VCD, $S_{glc}$ at steady state using perfusion $D_p$ and bleed $D_b$).
    *   *Interactive Example:* Open-loop simulation. Conceptual NMPC for achieving a target VCD.

**Part 6: Data-Driven Modeling for MPC – Learning from Data**

*   **Notebook 6.1: Introduction to Data-Driven Models in MPC**
    *   Recap: Why data-driven models? When are they useful?
    *   Overview of ANNs, PINNs, GPs as alternatives/complements to first-principles models.
    *   Setting up for ML libraries (TensorFlow/Keras or PyTorch, GPy/GPflow).

*   **Notebook 6.2: MPC with Artificial Neural Network (ANN) Models (ANN-MPC)**
    *   **Modeling:**
        *   Generating training data from a (known) nonlinear system (e.g., CSTR, bioreactor snippet).
        *   Training a Feedforward NN (or simple RNN/LSTM) to predict one-step-ahead dynamics: $\hat{x}_{k+1} = f_{ANN}(x_k, u_k)$.
        *   Validating the ANN model.
    *   **MPC Implementation:**
        *   Integrating the trained ANN into an NMPC framework using CasADi (leveraging its ability to handle external functions and AD).
        *   Predicting over $N_p$ steps by recursively calling the ANN.
        *   Formulating and solving the NMPC problem.
    *   *Interactive Example:* ANN-MPC for a CSTR or a simplified pendulum. Comparing performance to NMPC with the true model (if simple enough) or LMPC. Discussing computational aspects and extrapolation.

*   **Notebook 6.3: MPC with Physics-Informed Neural Network (PINN) Models (PINN-MPC)**
    *   **Modeling:**
        *   Choosing a simple system with known ODEs (e.g., damped oscillator, simple reaction).
        *   Formulating the PINN loss function ($L_{data} + L_{physics} + L_{bc/ic}$).
        *   Training the PINN to learn the solution trajectory and/or system parameters. (This is a significant modeling exercise in itself).
        *   Validating the PINN model.
    *   **MPC Implementation:**
        *   Using the trained PINN (which approximates $x(t)$ or $x_{k+1} = f_{PINN}(x_k, u_k)$) within an NMPC framework.
        *   Discussing the advantages (physical consistency, data efficiency) in the context of control.
    *   *Interactive Example:* PINN-MPC for the chosen simple system. Highlighting how the PINN model behaves, especially if data is sparse.

*   **Notebook 6.4: MPC with Gaussian Process (GP) Models (GP-MPC)**
    *   **Modeling:**
        *   Generating training data from a nonlinear system.
        *   Training a GP model (e.g., using GPy or GPflow) to learn $x_{k+1} = f_{GP}(x_k, u_k)$.
        *   Choosing a kernel and optimizing hyperparameters.
        *   Visualizing the GP's mean prediction and confidence intervals (variance).
    *   **MPC Implementation (Conceptual/Simplified for basic notebook):**
        *   Using the GP mean function $\mu_{GP}(x_k, u_k)$ for prediction in NMPC.
        *   Discussing how to propagate uncertainty (analytically difficult, often approximated or simplified for basic examples, e.g., assuming variance adds up or using Monte Carlo for illustration).
        *   Introduction to Chance Constraints: How the predictive variance $\sigma_{GP}^2$ can be used to make control decisions more robust (e.g., $g(x) + \beta \sigma_{GP}(x) \le 0$).
    *   *Interactive Example:* GP-MPC for a 1D or 2D system where uncertainty visualization is easier. Show how control actions might differ when considering model uncertainty.

**Part 7: Conclusion and Further Exploration**

*   **Notebook 7.1: Series Summary, Next Steps & Links to Advanced Topics**
    *   Recap of the journey from LMPC to NMPC and data-driven MPC.
    *   Brief discussion and links to resources/code for topics not covered in detail (Hybrid MPC, Distributed MPC, advanced robust NMPC, MHE, etc.).
    *   Ideas for further projects and self-study.

**Structure of Each Notebook:**

1.  **Learning Objectives.**
2.  **Theoretical Background:** Concise summary (from Appendices A, B, E).
3.  **Data Generation/Loading.**
4.  **Model Training & Validation (ANN, PINN, GP specific).**
5.  **MPC Formulation with the Data-Driven Model.**
6.  **Code Implementation of MPC Loop.**
7.  **Interactive Examples & Visualizations.**
8.  **Discussion:** Pros, cons, computational aspects, specific challenges of this data-driven approach in MPC.
9.  **Exercises/Challenges.**
10. **Key Takeaways.**

---