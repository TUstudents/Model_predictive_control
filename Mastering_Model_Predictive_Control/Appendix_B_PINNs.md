## Appendix B: Physics-Informed Neural Networks (PINNs) for Control-Oriented Modeling – Blending Data and Dynamics

Traditional data-driven models, like the Artificial Neural Networks (ANNs) discussed in Appendix A, learn entirely from observed data. While powerful, they often lack physical consistency, struggle with extrapolation, and may require vast amounts of data. On the other hand, first-principles models embody our understanding of underlying physical laws but might be incomplete or contain uncertain parameters. **Physics-Informed Neural Networks (PINNs)** emerge as a compelling hybrid approach, seeking to bridge this gap by integrating physical laws (typically expressed as differential equations) directly into the learning process of a neural network. This appendix explores the concept of PINNs and their potential for creating more robust, generalizable, and data-efficient models suitable for advanced control applications like Model Predictive Control.

### B.1 Limitations of Purely Data-Driven Models and the Need for Physics

While ANNs can be excellent interpolators, their "black-box" nature and reliance solely on data lead to several limitations when modeling physical or biological systems for control:

1.  **Lack of Physical Consistency:** A purely data-driven ANN has no inherent knowledge of fundamental physical laws (e.g., conservation of mass, energy). Its predictions might violate these laws, especially when extrapolating or dealing with noisy data.
2.  **Poor Extrapolation:** ANNs often fail to generalize well outside the domain of their training data. This is a significant issue for control, as MPC might explore regions not extensively covered during training.
3.  **Large Data Requirements:** To learn complex dynamics accurately, ANNs typically need large, diverse datasets, which may be expensive or impractical to obtain for many industrial or biological systems.
4.  **Interpretability Issues:** The "black-box" nature makes it hard to understand *why* the model makes certain predictions or to diagnose failures.

Incorporating known physics can help alleviate these issues, leading to models that are more robust, require less data, and behave more predictably.

### B.2 Introduction to Physics-Informed Neural Networks (PINNs)

The core idea of a PINN is to train a neural network not only to fit observed data points but also to satisfy given physical laws, typically expressed as Ordinary Differential Equations (ODEs) or Partial Differential Equations (PDEs).

**How PINNs Work:**

1.  **Neural Network as a Solution Approximator:**
    *   An ANN, typically a feedforward network, is used to approximate the solution of the differential equation. For example, if we are modeling a state $x(t)$ governed by an ODE, the ANN $\mathcal{N}(t; \theta)$ takes time $t$ (and possibly other input parameters or initial conditions) as input and outputs an approximation of $x(t)$, denoted $\hat{x}(t)$. The network parameters are $\theta$.

2.  **Automatic Differentiation for Derivatives:**
    *   A key enabler for PINNs is **Automatic Differentiation (AD)**. AD tools can compute exact derivatives of the neural network's output $\hat{x}(t)$ with respect to its input $t$ (i.e., $\frac{d\hat{x}}{dt}, \frac{d^2\hat{x}}{dt^2}$, etc.) and also with respect to parameters $\theta$. These derivatives are needed to form the residuals of the differential equations.

3.  **Physics-Informed Loss Function:**
    *   The loss function for training a PINN typically consists of several components:
        *   **Data Loss ($L_{data}$):** Measures the mismatch between the NN's predictions and observed data points (e.g., measurements of $x(t_i)$ at specific times $t_i$). This is the standard supervised learning loss.
            $L_{data} = \frac{1}{N_{data}} \sum_{i=1}^{N_{data}} || \hat{x}(t_i) - x_{data}^{(i)} ||^2$
        *   **Physics Residual Loss ($L_{physics}$):** Penalizes deviations of the NN's solution from satisfying the governing differential equation(s). For an ODE like $\frac{dx}{dt} = f(x, t, u; \lambda)$ (where $u$ are inputs and $\lambda$ are physical parameters):
            Let $r(t) = \frac{d\hat{x}}{dt} - f(\hat{x}(t), t, u; \lambda)$ be the residual.
            $L_{physics} = \frac{1}{N_{colloc}} \sum_{j=1}^{N_{colloc}} || r(t_j) ||^2$
            This loss is evaluated at a set of "collocation points" $t_j$ distributed throughout the domain of interest (these points don't need to have corresponding data values).
        *   **Boundary/Initial Condition Loss ($L_{bc/ic}$):** Enforces known boundary or initial conditions.
            $L_{bc/ic} = || \hat{x}(t_0) - x_0 ||^2$ (for an initial condition $x(t_0)=x_0$).

    *   **Total Loss:** The total loss is a weighted sum of these components:
        $L_{total} = w_{data}L_{data} + w_{physics}L_{physics} + w_{bc/ic}L_{bc/ic}$
        The weights $w_i$ are hyperparameters that balance the contribution of each loss term.

**(Figure B.1: Conceptual diagram of a PINN. An ANN outputs $\hat{x}(t)$. AD computes derivatives. These are used in $L_{data}$ (comparing to measurements) and $L_{physics}$ (checking DE residual at collocation points). The combined loss trains the ANN.)**

4.  **Training:** The neural network parameters $\theta$ are optimized (e.g., using Adam or L-BFGS) to minimize $L_{total}$.

### B.3 Formulating PINNs for Dynamic Systems in Control

For control applications, we are often interested in models of the form $\dot{x} = f_c(x, u, t; \lambda)$ or its discrete-time equivalent.

*   **System Identification with PINNs:**
    *   If the structure of $f_c$ is known but some physical parameters $\lambda$ (e.g., reaction rates, heat transfer coefficients) are unknown, these parameters can be included as trainable variables alongside the NN weights $\theta$. The PINN attempts to learn $\lambda$ such that the DEs are satisfied while matching data.
    *   The NN can approximate the state trajectories $x(t)$. Input $u(t)$ is typically a known function or measured data.

*   **Learning Surrogate Models with PINNs:**
    *   Even if $f_c$ is fully known, a PINN can be trained to act as a fast surrogate for a complex $f_c$ (e.g., a high-dimensional PDE model). The trained PINN $\mathcal{N}(t, x_0, u(\cdot); \theta)$ could then predict $x(t)$ much faster than numerically solving the original $f_c$.

*   **Discrete-Time PINNs:**
    *   While PINNs are often formulated for continuous-time DEs, the concept can be adapted for discrete-time difference equations $x_{k+1} = f_d(x_k, u_k; \lambda)$.
    *   The physics loss would then be based on the residual $x_{k+1,pred} - f_d(x_{k,pred}, u_k; \lambda)$, where $x_{k,pred}$ and $x_{k+1,pred}$ are outputs of an NN (e.g., an RNN or a feedforward net predicting step-wise).

### B.4 Advantages of PINNs for Control-Oriented Models

1.  **Improved Generalization and Data Efficiency:**
    *   By being constrained by physical laws, PINNs can often generalize better from smaller datasets compared to purely data-driven ANNs. The physics provides a strong inductive bias.
2.  **Physically Plausible Predictions:**
    *   The learned solutions are more likely to respect the underlying physics, even in regions with sparse data. This reduces the chance of "unphysical" predictions that a pure ANN might make.
3.  **Parameter Discovery (System Identification):**
    *   PINNs offer a powerful framework for estimating unknown physical parameters $\lambda$ within the DEs by including them as trainable variables.
4.  **Handling Noisy and Sparse Data:**
    *   The physics residual acts as a regularizer, making PINNs more robust to noise in the training data. They can also "fill in gaps" where data is sparse by relying on the DE structure.
5.  **Smoother and More Differentiable Models:**
    *   Since the NN is trained to satisfy differential equations, its output (the solution approximation) is inherently differentiable (as many times as required by the DE order, thanks to AD). This is beneficial for NMPC solvers that require smooth gradients.

### B.5 Using PINN-Derived Models in MPC

Once a PINN is trained and validated, the resulting neural network $\mathcal{N}(\cdot; \theta^*, \lambda^*)$ can be used as the predictive model within an NMPC framework, similar to how a standard ANN model is used (Appendix A):

$\hat{x}_{k+1|k} = \mathcal{N}_{discrete}(\hat{x}_{k|k}, u_{k|k}; \theta^*, \lambda^*)$
(where $\mathcal{N}_{discrete}$ represents the one-step prediction using the trained PINN, possibly after discretizing a continuous-time PINN output).

The advantages for MPC are:
*   **More Reliable Predictions:** Especially if the MPC explores regions outside the dense parts of the training data.
*   **Potential for Online Adaptation:** If parameters $\lambda$ are being estimated, an adaptive MPC scheme could use updated $\lambda$ values.
*   **Reduced Need for "Black-Box" Regularization:** The physics itself acts as a regularizer.

### B.6 Challenges and Considerations for PINNs

1.  **Tuning Loss Weights ($w_{data}, w_{physics}, w_{bc/ic}$):**
    *   Balancing the different loss terms is crucial and can be tricky. If $w_{physics}$ is too small, the NN might just fit the data; if too large, it might ignore noisy data to satisfy the DEs perfectly. Adaptive weighting schemes are an area of research.
2.  **Computational Cost of Training:**
    *   Training PINNs can be computationally intensive, as it involves evaluating the DE residuals (which require AD) at many collocation points in addition to the data points.
3.  **Choice of Collocation Points:**
    *   The distribution and number of collocation points can affect training performance and accuracy.
4.  **Complexity of DEs:**
    *   Very stiff DEs or highly chaotic systems can still be challenging for PINNs to learn accurately.
5.  **Software and Implementation:**
    *   Requires frameworks that seamlessly integrate ANNs with automatic differentiation (e.g., TensorFlow, PyTorch, JAX) and provide robust optimizers.

*Bioreactor Link (PINN-MPC):*
*   Consider a bioreactor model (Chapter 4) where the general mass balance structure is known, but specific kinetic parameters ($\mu_{max}, K_S, Y_{X/S}$) are uncertain or vary.
*   A PINN could be formulated where an ANN approximates $X_v(t), S_{glc}(t), P_{mab}(t)$, etc. The loss function would include:
    *   $L_{data}$: Mismatch with infrequent offline measurements of $X_v, S_{glc}, P_{mab}$.
    *   $L_{physics}$: Residuals of the ODEs (e.g., $\frac{d\hat{X_v}}{dt} - (\hat{\mu} - \hat{\mu_d})\hat{X_v} + \frac{F}{V}\hat{X_v}$) evaluated at many collocation points in time.
    *   $\mu_{max}, K_S, Y_{X/S}$ could be included as trainable parameters $\lambda$.
*   The trained PINN would provide a model that is consistent with both the observed data and the known biological rate structures, potentially leading to more robust NMPC performance, especially if parameters drift or batches show variability. This could also help in identifying how parameters change from batch to batch.

**In conclusion, PINNs represent a significant advancement in scientific machine learning, offering a powerful way to synergize data-driven learning with domain knowledge embodied in physical laws. For control-oriented modeling, particularly for complex nonlinear systems encountered in MPC, PINNs hold the promise of creating more accurate, robust, and data-efficient models. While still an active area of research with its own set of challenges, their ability to produce physically consistent and generalizable surrogate models makes them a highly attractive option for the future of advanced process control.**

---