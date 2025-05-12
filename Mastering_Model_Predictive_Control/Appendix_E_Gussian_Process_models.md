## Appendix E: Gaussian Process Models for MPC – A Bayesian Approach to Learning Dynamics with Uncertainty

Previous appendices explored Artificial Neural Networks (ANNs) and Physics-Informed Neural Networks (PINNs) as data-driven modeling tools for MPC. **Gaussian Processes (GPs)** offer another compelling non-parametric, Bayesian method for learning system dynamics from data. A key advantage of GPs is their inherent ability to provide not just predictions but also a principled measure of uncertainty about those predictions. This uncertainty information can be invaluable for robust MPC design and active learning. This appendix introduces Gaussian Processes and discusses their application in GP-MPC.

### E.1 Introduction to Gaussian Processes (GPs)

A Gaussian Process is a collection of random variables, any finite number of which have a joint Gaussian distribution. In the context of regression or modeling, a GP defines a **distribution over functions**. Instead of learning specific parameters of a fixed functional form (like in linear regression or standard ANNs), a GP directly models the function itself as a random variable.

*   **Intuition:** Imagine you have a few data points representing a function. A GP doesn't just try to fit *a* curve through these points; it considers *all possible* smooth curves that are consistent with the data, assigning higher probabilities to curves that are "more likely" given prior assumptions about the function's behavior (e.g., smoothness).
*   **Non-parametric:** The complexity of the model grows with the amount of data, rather than being fixed by a pre-defined number of parameters.
*   **Bayesian:** GPs naturally incorporate prior beliefs about the function and update these beliefs based on observed data to produce a posterior distribution over functions.

### E.2 Key Components of a Gaussian Process

A Gaussian Process $f(x)$ is completely specified by its **mean function** $m(x)$ and its **covariance function** (or kernel) $k(x, x')$:

$f(x) \sim \mathcal{GP}(m(x), k(x, x'))$

1.  **Mean Function ($m(x)$):**
    *   $m(x) = E[f(x)]$
    *   Represents the prior expectation of the function value at input $x$, before any data is observed.
    *   Often, $m(x)$ is set to zero if there's no strong prior belief, or it can be a simple parametric function (e.g., linear). The GP then learns deviations from this mean.

2.  **Covariance Function (Kernel, $k(x, x')$):**
    *   $k(x, x') = E[(f(x) - m(x))(f(x') - m(x'))]$
    *   This is the heart of the GP. It defines the covariance between the function values at any two input points $x$ and $x'$.
    *   The kernel encodes prior assumptions about the function's properties, such as:
        *   **Smoothness:** How rapidly the function is expected to change.
        *   **Lengthscale:** How far apart inputs $x$ and $x'$ can be before their function values become uncorrelated.
        *   **Periodicity:** If the function is expected to be periodic.
        *   **Signal Variance:** The overall variability of the function from its mean.
    *   **Common Kernels:**
        *   **Radial Basis Function (RBF) / Squared Exponential (SE) Kernel:**
            $k_{SE}(x, x') = \sigma_f^2 \exp\left(-\frac{||x - x'||^2}{2l^2}\right)$
            Produces infinitely smooth functions. $\sigma_f^2$ is the signal variance, $l$ is the lengthscale.
        *   **Matérn Kernels (e.g., Matérn 3/2, Matérn 5/2):**
            $k_{Matern3/2}(x,x') = \sigma_f^2 \left(1 + \frac{\sqrt{3}||x-x'||}{l}\right) \exp\left(-\frac{\sqrt{3}||x-x'||}{l}\right)$
            Allow for tunable smoothness (functions are $k$ times differentiable if using Matérn $2k+1/2$). Often more realistic for physical systems than the infinitely smooth RBF.
        *   **Linear Kernel:** $k_{LIN}(x, x') = \sigma_b^2 + \sigma_v^2 (x - c)(x' - c)$.
        *   **Periodic Kernel:** For functions with known periodicity.
        *   Kernels can be combined (added, multiplied) to create more complex covariance structures.
    *   The kernel contains **hyperparameters** (e.g., $\sigma_f^2, l$) that are typically learned from the data.

### E.3 Gaussian Process Regression (Prediction)

Given a set of $N$ training data points $\mathcal{D} = \{ (x_i, y_i) \}_{i=1}^N$, where $y_i = f(x_i) + \epsilon_i$ and $\epsilon_i \sim \mathcal{N}(0, \sigma_n^2)$ is i.i.d. Gaussian noise with variance $\sigma_n^2$. We want to predict the function value $f^*$ at a new test point $x^*$.

1.  **Joint Gaussian Distribution:** According to the GP definition, the training outputs $\mathbf{y} = [y_1, \dots, y_N]^T$ and the test output $f^*$ have a joint Gaussian distribution:
    $\begin{bmatrix} \mathbf{y} \\ f^* \end{bmatrix} \sim \mathcal{N} \left( \begin{bmatrix} \mathbf{m}(X) \\ m(x^*) \end{bmatrix}, \begin{bmatrix} K(X, X) + \sigma_n^2 I & K(X, x^*) \\ K(x^*, X) & k(x^*, x^*) \end{bmatrix} \right)$
    Where:
    *   $X$ is the matrix of training inputs.
    *   $\mathbf{m}(X)$ is the vector of prior means at training inputs.
    *   $K(X, X)$ is the $N \times N$ covariance matrix with entries $K_{ij} = k(x_i, x_j)$.
    *   $K(X, x^*)$ is an $N \times 1$ vector with entries $k(x_i, x^*)$.
    *   $K(x^*, X) = K(X, x^*)^T$.
    *   $k(x^*, x^*)$ is the prior variance at $x^*$.

2.  **Predictive Distribution:** Using standard formulas for conditional Gaussian distributions, the posterior predictive distribution $p(f^* | x^*, \mathcal{D})$ is also Gaussian:
    $f^* | x^*, \mathcal{D} \sim \mathcal{N}(\mu_{GP}(x^*), \sigma_{GP}^2(x^*))$
    With predictive mean:
    $\mu_{GP}(x^*) = m(x^*) + K(x^*, X) [K(X, X) + \sigma_n^2 I]^{-1} (\mathbf{y} - \mathbf{m}(X))$
    And predictive variance:
    $\sigma_{GP}^2(x^*) = k(x^*, x^*) - K(x^*, X) [K(X, X) + \sigma_n^2 I]^{-1} K(X, x^*)$

*Feynman Insight:* The predictive mean $\mu_{GP}(x^*)$ can be seen as a correction to the prior mean $m(x^*)$, where the correction is a linear combination of the training residuals $(\mathbf{y} - \mathbf{m}(X))$. The weights in this combination depend on how similar $x^*$ is to each training input $x_i$ (via $K(x^*, X)$) and the inter-correlations between training inputs (via $[K(X, X) + \sigma_n^2 I]^{-1}$). The predictive variance $\sigma_{GP}^2(x^*)$ starts from the prior variance $k(x^*, x^*)$ and is reduced by an amount depending on how much information the training data provides about $x^*$. The variance is typically smaller near training data points and larger further away.

### E.4 Hyperparameter Optimization (Training a GP)

The kernel hyperparameters (e.g., $l, \sigma_f^2$) and the noise variance $\sigma_n^2$ are usually unknown and must be learned from the data. This is typically done by maximizing the **log marginal likelihood** (also called evidence) of the training data $\mathbf{y}$ given the inputs $X$ and hyperparameters $\theta_{hyp} = \{l, \sigma_f^2, \sigma_n^2, \dots \}$:

$\log p(\mathbf{y} | X, \theta_{hyp}) = -\frac{1}{2} (\mathbf{y} - \mathbf{m}(X))^T [K(X, X) + \sigma_n^2 I]^{-1} (\mathbf{y} - \mathbf{m}(X)) - \frac{1}{2} \log |K(X, X) + \sigma_n^2 I| - \frac{N}{2} \log(2\pi)$

This expression balances data fit (first term) with model complexity (second term, which penalizes overly complex kernels that lead to near-singular covariance matrices). Gradient-based optimization methods (e.g., L-BFGS, conjugate gradient) are used to find the $\theta_{hyp}$ that maximize this log marginal likelihood.

### E.5 Advantages of Gaussian Processes for Modeling

1.  **Principled Uncertainty Quantification:** GPs naturally provide a predictive variance $\sigma_{GP}^2(x^*)$, which quantifies the model's confidence in its prediction. This is crucial for robust control and decision-making.
2.  **Non-parametric Flexibility:** GPs can capture complex, non-linear functions without requiring a pre-specified parametric form.
3.  **Good Performance with Small Datasets:** Often, GPs can provide reasonable models even with limited training data, especially when prior knowledge is encoded in the kernel.
4.  **Incorporation of Prior Knowledge:** The choice of mean function and kernel structure allows for explicit encoding of prior beliefs about the function (e.g., smoothness, periodicity).
5.  **Analytically Tractable (for basic GP regression):** The predictive mean and variance have closed-form expressions.

### E.6 Disadvantages and Challenges of Gaussian Processes

1.  **Computational Complexity:**
    *   Standard GP regression involves inverting the $N \times N$ matrix $[K(X, X) + \sigma_n^2 I]$, which takes $O(N^3)$ time for training (hyperparameter optimization usually involves multiple such inversions).
    *   Prediction at a new point takes $O(N)$ or $O(N^2)$ depending on the implementation.
    *   This makes standard GPs computationally challenging for datasets with $N > \text{a few thousands}$.
2.  **Choice of Kernel:** Selecting an appropriate kernel and setting appropriate priors for its hyperparameters can be non-trivial and may require domain expertise or extensive cross-validation.
3.  **High-Dimensional Inputs:** Standard GPs can suffer from the "curse of dimensionality" if the input space $x$ is very high-dimensional, as the amount of data needed to cover the space grows exponentially. (Techniques like Automatic Relevance Determination - ARD, using separate lengthscales for each input dimension, can help somewhat).

### E.7 Gaussian Process MPC (GP-MPC)

GPs can be integrated into an MPC framework, particularly NMPC, by using the GP as the predictive model.

1.  **Using GPs as the Predictive Model:**
    *   **Learning System Dynamics:** A GP can be trained to model the one-step-ahead dynamics:
        $\hat{x}_{k+1|k} = \mu_{GP}(x_k, u_k) + w_k$
        where $w_k \sim \mathcal{N}(0, \sigma_{GP}^2(x_k, u_k))$ captures the GP's predictive uncertainty.
    *   **Learning Model Error/Residuals:** A GP can learn the error of a nominal (e.g., first-principles or linearized) model:
        $x_{k+1} = f_{nominal}(x_k, u_k) + f_{GP,error}(x_k, u_k) + \text{noise}$
        This hybrid approach leverages existing knowledge and uses the GP to capture unmodeled dynamics.

2.  **Propagating Uncertainty in Predictions for MPC:**
    *   When predicting multiple steps ahead ($N_p > 1$), the input to the GP at step $j+1$ (i.e., $\hat{x}_{k+j|k}$) is itself a random variable with a mean and variance from the GP prediction at step $j$.
    *   Propagating this uncertainty through the nonlinear GP mean function and updating the variance is analytically intractable.
    *   **Approximation Methods:**
        *   **Taylor Series Expansion / Moment Matching:** Approximate the GP output distribution by linearizing the mean function.
        *   **Monte Carlo Sampling:** Sample trajectories through the prediction horizon. Computationally expensive.
        *   **Unscented Transform:** Similar to UKF, use sigma points to propagate mean and covariance.
    *   This results in predicted state trajectories that are themselves distributions (e.g., Gaussian approximations $\mathcal{N}(\hat{\mu}_{x,k+j|k}, \hat{\Sigma}_{x,k+j|k})$).

3.  **Robust MPC with GPs (e.g., Chance Constraints):**
    *   The propagated predictive uncertainty $(\hat{\Sigma}_{x,k+j|k}, \hat{\Sigma}_{y,k+j|k})$ can be used to formulate probabilistic or chance constraints in the MPC optimization:
        $P(\text{constraint } g(\hat{x}_{k+j|k}, u_{k+j|k}) \le 0) \ge 1-\delta$
        This aims to satisfy constraints with a high probability $\delta$.
    *   This can lead to less conservative control actions compared to worst-case robust MPC, as it explicitly uses the model's confidence.

4.  **Active Learning / Exploration in GP-MPC:**
    *   The GP's predictive variance indicates regions of high model uncertainty. The MPC objective can be augmented with an "exploration" term that encourages control actions leading to these uncertain regions, with the goal of collecting more informative data to improve the GP model online. This balances exploitation (achieving control objectives) and exploration (improving the model).

### E.8 Sparse GPs and Other Approximations for Scalability

To address the $O(N^3)$ complexity, various sparse GP approximations exist:
*   They typically use a smaller set of $M \ll N$ "inducing points" to summarize the information from the full dataset.
*   Examples: FITC (Fully Independent Training Conditional), VFE (Variational Free Energy), DTC (Deterministic Training Conditional).
*   Deep Gaussian Processes: Stack multiple GP layers, allowing for more complex function representation and potentially better scalability.

These approximations make GPs applicable to larger datasets encountered in some MPC scenarios.

*Bioreactor Link (GP-MPC):*
*   **Modeling Uncertain Kinetics:** If specific growth rates or yield coefficients in a bioreactor model are uncertain or known to vary (e.g., due to unmeasured raw material variations or cell line evolution), a GP can be trained to model these terms or their deviations from a nominal value.
*   **Batch-to-Batch Variability:** A GP could learn how initial conditions or specific events in previous batches influence the dynamics of the current batch.
*   **Soft Sensors with Uncertainty:** GPs can be used to build soft sensors (e.g., predicting VCD from online capacitance and OUR/CER) that also provide a confidence level for their estimates.
*   **Safer Operation Near Constraints:** The GP's uncertainty can be used in a chance-constrained MPC to operate closer to optimal constraints (e.g., minimum glucose level) while maintaining a desired level of safety. For example, the controller might be more conservative if the GP model is highly uncertain about the glucose dynamics near the lower limit.

**In summary, Gaussian Processes provide a statistically sound and flexible framework for data-driven modeling, distinguished by their inherent uncertainty quantification. When integrated into an MPC framework, GP-MPC offers exciting possibilities for robust control, active learning, and decision-making under uncertainty. While computational scalability and kernel selection remain key considerations, the ability of GPs to learn from limited data and provide confidence estimates makes them a valuable tool in the advanced control practitioner's workbench, especially for complex systems where first-principles models are incomplete or data is precious.**

---