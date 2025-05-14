## Mastering Model Predictive Control: From Foundations to Frontier Applications

**Table of Contents**


**Part I: Fundamentals of Predictive Control**

*   **Chapter 1: Introduction to Advanced Control & The MPC Paradigm**
    *   1.1 The Limits of Classical Control: Why We Need More "Foresight"
        *   1.1.1 Single-Input Single-Output (SISO) Focus vs. Multi-Input Multi-Output (MIMO) Reality
        *   1.1.2 Handling Constraints: The "Rules of the Road"
        *   1.1.3 Time Delays and Non-Minimum Phase Systems: "Lag and Unexpected Turns"
        *   1.1.4 Optimal Performance Beyond Simple Regulation
    *   1.2 The MPC Philosophy: Control by Optimization and Prediction
        *   1.2.1 Prediction Using a Model ("Looking into the Future")
        *   1.2.2 Optimization of Control Actions ("Making Optimal Decisions")
        *   1.2.3 Explicit Constraint Handling ("Respecting the Rules of the Road")
        *   1.2.4 Receding Horizon Principle ("Re-evaluating and Adapting")
    *   1.3 Historical Context and Evolution of MPC
    *   1.4 Overview of MPC Applications
    *   1.5 Prerequisites: Gearing Up for the Journey

*   **Chapter 2: Foundations of Linear MPC: The Workhorse**
    *   2.1 Discrete-Time Linear State-Space Models: The Language of Prediction
    *   2.2 Prediction Equations: Peering into the Future, Step by Step
        *   2.2.1 Compact Matrix Formulation
        *   2.2.2 The Control Horizon ($N_c < N_p$)
    *   2.3 Objective Function Formulation: Defining "Good" Performance
        *   2.3.1 Tracking Error Term
        *   2.3.2 Control Effort Term
        *   2.3.3 Control Move Suppression Term
        *   2.3.4 Vectorized Form of the Objective Function
    *   2.4 Constraint Handling: Operating Within Bounds
        *   2.4.1 Input Constraints
        *   2.4.2 Input Rate Constraints
        *   2.4.3 Output Constraints
        *   2.4.4 Combined Linear Inequality Form
    *   2.5 The Quadratic Program (QP): Solving for the Optimal Moves
    *   2.6 The Receding Horizon Strategy in LMPC: Plan, Act, Re-plan
    *   2.7 Illustrative Example: Double Integrator

**Part II: Modeling and Estimation for MPC**

*   **Chapter 3: Process Modeling for Control: Principles and Linear Models**
    *   3.1 The Role of Models in Control: MPC's "Crystal Ball"
    *   3.2 Modeling Paradigms: A Spectrum of Approaches
        *   3.2.1 First-Principles (Mechanistic or White-Box) Models
        *   3.2.2 Empirical (Data-Driven or Black-Box) Models
        *   3.2.3 Hybrid (Grey-Box) Models
        *   3.2.4 Continuous-Time vs. Discrete-Time Models
    *   3.3 Linear System Identification Techniques for LMPC
        *   3.3.1 Key Steps in System Identification
        *   3.3.2 Common Linear Model Structures
    *   3.4 Input Signal Design for Identification: "Asking the Right Questions"
    *   3.5 Model Validation and Selection: "Trust, but Verify"
    *   3.6 Model Uncertainty: Acknowledging Imperfection
    *   3.7 Practical Example: Identifying a Linear Model for a Thermal Process

*   **Chapter 4: Nonlinear and Bioprocess Modeling for Advanced Control**
    *   4.1 Introduction to Nonlinear Systems Modeling: Beyond Straight Lines
    *   4.2 First-Principles Nonlinear Modeling for Bioprocesses: Capturing the Biology
        *   4.2.1 Common Components of a Bioreactor Model
        *   4.2.2 Kinetic Rate Expressions (The Nonlinear Core)
        *   4.2.3 Challenges in First-Principles Bioprocess Modeling
    *   4.3 Data-Driven Nonlinear Modeling Approaches
        *   4.3.1 Block-Oriented Models
        *   4.3.2 Volterra Series Models
        *   4.3.3 Artificial Neural Networks (ANNs)
        *   4.3.4 Gaussian Processes (GPs)
        *   4.3.5 Physics-Informed Neural Networks (PINNs)
    *   4.4 Model Reduction and Simplification for NMPC
    *   4.5 Example: Step-by-Step Development of a Fed-Batch Bioreactor Model

*   **Chapter 5: State Estimation and Disturbance Handling in MPC: Knowing Where You Are and What's Pushing You**
    *   5.1 The Need for State Estimation: When You Can't See Everything
    *   5.2 Observers for Linear Systems: The Luenberger Observer
    *   5.3 Kalman Filtering (KF): Optimal Estimation under Gaussian Noise
    *   5.4 Estimators for Nonlinear Systems: EKF, UKF, and MHE
        *   5.4.1 Extended Kalman Filter (EKF)
        *   5.4.2 Unscented Kalman Filter (UKF)
        *   5.4.3 Moving Horizon Estimation (MHE)
    *   5.5 Disturbance Modeling and Rejection: Dealing with the Unknown
        *   5.5.1 Types of Disturbances
        *   5.5.2 Integrating Disturbances into the Prediction Model
        *   5.5.3 Integral Action for Offset-Free Tracking
    *   5.6 Practical Considerations for Bioreactors

**Part III: Core MPC Algorithms and Properties**

*   **Chapter 6: Optimization Algorithms for MPC: The Engine Room**
    *   6.1 Fundamentals of Mathematical Optimization: A Quick Tour
        *   6.1.1 Key Concepts (Feasible Set, Minima, Convexity)
        *   6.1.2 Optimality Conditions (KKT Conditions)
        *   6.1.3 Duality
    *   6.2 Quadratic Programming (QP) for Linear MPC: The Efficient Workhorse
        *   6.2.1 Active Set Methods
        *   6.2.2 Interior Point Methods (IPMs)
        *   6.2.3 Computational Complexity and Warm-Starting
    *   6.3 Nonlinear Programming (NLP) for Nonlinear MPC: Tackling Complexity
        *   6.3.1 Challenges with NLPs
        *   6.3.2 Sequential Quadratic Programming (SQP)
        *   6.3.3 Interior Point Methods for NLP
        *   6.3.4 Automatic Differentiation (AD)
    *   6.4 Discretization Methods for Continuous-Time Optimal Control (NMPC)
        *   6.4.1 Direct Single Shooting
        *   6.4.2 Direct Multiple Shooting
        *   6.4.3 Direct Collocation Methods
    *   6.5 Software Tools and Libraries: The Implementer's Toolkit
    *   6.6 Real-Time Feasibility and Computational Demands

*   **Chapter 7: Stability and Robustness in MPC: Ensuring Reliable Performance**
    *   7.1 The Challenge: Finite Horizon vs. Infinite Horizon Performance
    *   7.2 Nominal Stability of Linear MPC: The Role of Terminal Ingredients
        *   7.2.1 Terminal Cost Function
        *   7.2.2 Terminal Constraint Set
        *   7.2.3 Sufficiently Long Prediction Horizon
    *   7.3 Feasibility and Recursive Feasibility: Can We Always Find a Solution?
    *   7.4 Robust MPC Design: Handling Model Uncertainty
        *   7.4.1 Min-Max MPC (Worst-Case Optimization)
        *   7.4.2 Tube-Based MPC
        *   7.4.3 Explicit MPC / Multi-Parametric MPC
        *   7.4.4 Offset-Free Tracking (Revisited)
    *   7.5 Tuning Horizons ($N_p, N_c$) and Weights ($\mathbf{Q}, \mathbf{R}, \mathbf{S}$): Practical Aspects

*   **Chapter 8: Nonlinear Model Predictive Control (NMPC): Embracing Complexity**
    *   8.1 Motivation for NMPC: When Straight Lines Aren't Enough
    *   8.2 NMPC Problem Formulation: The NLP at its Core
    *   8.3 Computational Challenges of NMPC
    *   8.4 Numerical Solution Strategies for NMPC (Revisiting from Chapter 6)
    *   8.5 State Estimation for NMPC (Revisiting EKF, UKF, MHE)
    *   8.6 Example: NMPC for a Continuous Stirred Tank Reactor (CSTR)

**Part IV: Advanced MPC Techniques and Applications**

*   **Chapter 9: Advanced Topics in NMPC and Hybrid Systems: Pushing the Boundaries**
    *   9.1 Stability of Nonlinear MPC: Ensuring Long-Term Good Behavior
    *   9.2 Robust Nonlinear MPC: Handling Uncertainty in Nonlinear Systems
    *   9.3 Economic NMPC (EMPC) / Dynamic Real-Time Optimization (DRTO)
    *   9.4 Learning-Based MPC / Adaptive MPC: Learning from Experience
    *   9.5 Hybrid MPC: Controlling Systems with Continuous and Discrete Behavior
        *   9.5.1 Modeling Hybrid Systems (MLD, PWA)
        *   9.5.2 Hybrid MPC (MIQP, MINLP)

*   **Chapter 10: Case Study: NMPC for Fed-Batch Bioreactor Optimization – Maximizing Productivity**
    *   10.1 Process Description: Mammalian Cell Culture for mAb Production
    *   10.2 Control Objectives: Beyond Simple Setpoint Tracking
    *   10.3 Detailed Model Development (Illustrative - Cell Line Specific)
    *   10.4 Manipulated Variables and Process Constraints
    *   10.5 State and Parameter Estimation Strategy
    *   10.6 NMPC/EMPC Formulation
    *   10.7 Implementation Details and Practical Considerations
    *   10.8 Simulation Results and Comparison
    *   10.9 Discussion: Benefits, Challenges, and Future Outlook

*   **Chapter 11: Case Study: MPC for Continuous Biomanufacturing – Steering Perfusion Systems to Steady Productivity**
    *   11.1 Introduction to Continuous Biomanufacturing and Perfusion Systems
    *   11.2 Control Objectives in Perfusion Systems
    *   11.3 Process Modeling for Perfusion Systems
    *   11.4 Manipulated Variables and Key Constraints
    *   11.5 MPC Formulation for Setpoint Tracking and Disturbance Rejection
    *   11.6 Potential for Economic MPC in Optimizing Steady-State Operation
    *   11.7 Integration with PAT for Real-Time Monitoring and Control
    *   11.8 Simulation Results: Demonstrating Stability and Performance
    *   11.9 Discussion: The Future of Continuous Bioprocessing Control

**Part V: Implementation and Future Horizons**

*   **Chapter 12: Practical Implementation, Commissioning, and Future Frontiers of MPC: From Theory to Reality and Beyond**
    *   12.1 Software and Hardware Architecture for MPC Implementation
    *   12.2 Commissioning and Tuning an MPC Application
        *   12.2.1 Offline Simulation and Validation
        *   12.2.2 Pre-Commissioning Checks
        *   12.2.3 Initial Open-Loop "Shadow Mode" Operation
        *   12.2.4 First Closed-Loop Tests
        *   12.2.5 Iterative Tuning and Performance Monitoring
        *   12.2.6 Troubleshooting Common Issues
    *   12.3 Regulatory Considerations for MPC in Pharma/Biopharma (PAT)
    *   12.4 Research Frontiers and Future Outlook for MPC
        *   12.4.1 Large-Scale and Distributed MPC
        *   12.4.2 Stochastic MPC
        *   12.4.3 Event-Triggered and Self-Triggered MPC
        *   12.4.4 MPC for Cyber-Physical Systems (CPS) and Security
        *   12.4.5 Enhanced Integration with Artificial Intelligence (AI)
        *   12.4.6 Novel Application Areas
        *   12.4.7 Bioreactor/Bioprocessing Futures
    *   12.5 Open Challenges and Concluding Remarks: The Enduring Paradigm

**Appendices**

*   **Appendix A: MPC with Neural Network Models – Learning the Dynamics**
    *   A.1 Introduction to Artificial Neural Networks (ANNs) for Dynamic Systems
    *   A.2 Training ANNs for Dynamic System Identification
    *   A.3 Integrating ANN Models into the MPC Loop
    *   A.4 Advantages and Disadvantages of ANN-MPC
    *   A.5 Hybrid Approaches

*   **Appendix B: Physics-Informed Neural Networks (PINNs) for Control-Oriented Modeling – Blending Data and Dynamics**
    *   B.1 Limitations of Purely Data-Driven Models and the Need for Physics
    *   B.2 Introduction to Physics-Informed Neural Networks (PINNs)
    *   B.3 Formulating PINNs for Dynamic Systems in Control
    *   B.4 Advantages of PINNs for Control-Oriented Models
    *   B.5 Using PINN-Derived Models in MPC
    *   B.6 Challenges and Considerations for PINNs

*   **Appendix C: Review of Linear Algebra and Optimization Basics – The Mathematical Toolkit**
    *   C.1 Linear Algebra Essentials
        *   C.1.1 Vectors and Matrices
        *   C.1.2 Eigenvalues and Eigenvectors
        *   C.1.3 Singular Value Decomposition (SVD)
        *   C.1.4 Positive Definite and Semidefinite Matrices
    *   C.2 Optimization Basics
        *   C.2.1 Calculus for Optimization (Gradient, Hessian)
        *   C.2.2 Unconstrained Optimization
        *   C.2.3 Constrained Optimization and KKT Conditions
        *   C.2.4 Convex Optimization (LP, QP)

*   **Appendix D: Software Tools for MPC Simulation and Implementation – The Practitioner's Workbench**
    *   D.1 MATLAB: A Dominant Platform
    *   D.2 Python: A Versatile Open-Source Ecosystem
    *   D.3 Standalone Optimization Solvers (QP, NLP, MINLP/MIQP)
    *   D.4 Setting Up a Basic MPC Simulation Environment

*   **Appendix E: Gaussian Process Models for MPC – A Bayesian Approach to Learning Dynamics with Uncertainty**
    *   E.1 Introduction to Gaussian Processes (GPs)
    *   E.2 Key Components of a Gaussian Process (Mean Function, Kernel)
    *   E.3 Gaussian Process Regression (Prediction)
    *   E.4 Hyperparameter Optimization (Training a GP)
    *   E.5 Advantages of Gaussian Processes for Modeling
    *   E.6 Disadvantages and Challenges of Gaussian Processes
    *   E.7 Gaussian Process MPC (GP-MPC)
    *   E.8 Sparse GPs and Other Approximations for Scalability

*   **Appendix F: The Kalman Filter – Optimal State Estimation for Linear Systems with Noise**
    *   F.1 The Problem: State Estimation in the Presence of Noise
    *   F.2 The Kalman Filter Algorithm: A Two-Step Recursive Process
    *   F.3 Optimality and Properties of the Kalman Filter
    *   F.4 Tuning the Kalman Filter: The Role of $Q_K$ and $R_K$
    *   F.5 Connection to MPC
    *   F.6 Limitations and Extensions


    # Install python environmen

    Use a WSL or Linux environment/OS
    uv init --bare --python 3.12 was used to setup the pyproject.toml file.