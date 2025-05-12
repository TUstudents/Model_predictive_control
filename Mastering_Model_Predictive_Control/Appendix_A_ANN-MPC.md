## Appendix A: MPC with Neural Network Models – Learning the Dynamics

Model Predictive Control fundamentally relies on a process model to predict future behavior. While first-principles models offer deep insight and empirical linear models are computationally convenient, there are situations where developing either is challenging, or where the system exhibits complex, poorly understood nonlinearities. In such cases, **Artificial Neural Networks (ANNs)** offer a powerful data-driven approach to learn these dynamics directly from process data. This appendix explores how ANNs can be used as the predictive model within an MPC framework, a strategy often termed Neural Network MPC (NNMPC) or ANN-MPC.

### A.1 Introduction to Artificial Neural Networks (ANNs) for Dynamic Systems

Artificial Neural Networks are computational models inspired by the structure and function of biological neural networks. They consist of interconnected "neurons" or nodes, typically organized in layers: an input layer, one or more hidden layers, and an output layer.

*   **Feedforward Neural Networks (FNNs or Multi-Layer Perceptrons - MLPs):**
    *   Information flows in one direction, from input to output, through the hidden layers.
    *   Each neuron computes a weighted sum of its inputs, adds a bias, and then passes the result through a nonlinear **activation function** (e.g., sigmoid, tanh, ReLU - Rectified Linear Unit).
    *   With enough hidden neurons and appropriate activation functions, MLPs are **universal approximators**, meaning they can approximate any continuous function to an arbitrary degree of accuracy, given sufficient data and network complexity.
    *   For dynamic systems, an FNN can be used to model the one-step-ahead prediction:
        $\hat{x}_{k+1} = f_{ANN}(x_k, u_k; \theta)$ or $\hat{y}_{k+1} = f_{ANN}(y_k, \dots, y_{k-n_y}, u_k, \dots, u_{k-n_u}; \theta)$
        where $\theta$ represents the network's weights and biases.

*   **Recurrent Neural Networks (RNNs):**
    *   Contain feedback connections, allowing them to maintain an internal "memory" or state. This makes them naturally suited for modeling sequential data and dynamic systems.
    *   **Simple RNNs:** Can suffer from vanishing/exploding gradient problems during training, making it hard to learn long-term dependencies.
    *   **Long Short-Term Memory (LSTM) Networks and Gated Recurrent Units (GRUs):** Specialized RNN architectures with gating mechanisms designed to overcome these issues and effectively learn long-range dependencies. These are often preferred for complex time-series modeling.
    *   An RNN can directly model the state transition:
        $h_{k+1}, \hat{x}_{k+1} = f_{RNN}(x_k, u_k, h_k; \theta)$
        where $h_k$ is the hidden state of the RNN.

**(Figure A.1: Simple diagrams of (a) a Feedforward Neural Network and (b) a Recurrent Neural Network structure.)**

### A.2 Training ANNs for Dynamic System Identification

The process of "teaching" an ANN to represent the system dynamics involves:

1.  **Data Collection and Preprocessing:**
    *   **Data Requirements:** ANNs, especially deep ones, are data-hungry. Sufficient input-output data covering the expected operating range of the system is needed. The data should be persistently exciting.
    *   **Input Selection:** Choose inputs to the ANN that are relevant for predicting the desired outputs (e.g., past states/outputs, past inputs).
    *   **Normalization/Standardization:** Input and output data are typically scaled (e.g., to a [0,1] or [-1,1] range, or to have zero mean and unit variance) to improve training stability and speed.
    *   **Data Splitting:** Divide the data into training, validation, and test sets.
        *   Training set: Used to adjust the network weights $\theta$.
        *   Validation set: Used during training to monitor for overfitting and for hyperparameter tuning (e.g., number of layers, neurons per layer, learning rate).
        *   Test set: Used after training for a final, unbiased evaluation of the model's generalization performance.

2.  **Network Architecture Selection:**
    *   Number of hidden layers.
    *   Number of neurons in each hidden layer.
    *   Type of activation functions (e.g., ReLU is common for hidden layers, linear for output layers in regression tasks).
    *   Choice of network type (FNN, LSTM, GRU). This often involves some trial and error and domain knowledge.

3.  **Loss Function:**
    *   Quantifies the error between the ANN's predictions and the true target values.
    *   For regression (predicting continuous values like states or outputs), the **Mean Squared Error (MSE)** is common:
        $L(\theta) = \frac{1}{N_{samples}} \sum_{i=1}^{N_{samples}} || \hat{y}^{(i)} - y^{(i)} ||^2$
        where $\hat{y}^{(i)}$ is the network's prediction for sample $i$, and $y^{(i)}$ is the true value.

4.  **Optimization Algorithm (Training):**
    *   The goal is to find the weights $\theta$ that minimize the loss function $L(\theta)$.
    *   **Backpropagation:** An algorithm that efficiently computes the gradient of the loss function with respect to all network weights.
    *   **Gradient-Based Optimizers:**
        *   Stochastic Gradient Descent (SGD) and its variants (e.g., Adam, RMSprop) are commonly used to update the weights iteratively based on these gradients.
        *   Involves hyperparameters like learning rate, batch size.

5.  **Regularization and Overfitting Prevention:**
    *   **Overfitting:** Occurs when the network learns the training data too well, including its noise, and fails to generalize to new, unseen data.
    *   **Techniques:**
        *   **Early Stopping:** Monitor performance on the validation set during training and stop training when validation error starts to increase.
        *   **L1/L2 Regularization (Weight Decay):** Add a penalty term to the loss function based on the magnitude of the weights, discouraging overly complex models.
        *   **Dropout:** Randomly "drop" (set to zero) a fraction of neurons during each training iteration, forcing the network to learn more robust features.

### A.3 Integrating ANN Models into the MPC Loop

Once a suitable ANN model $f_{ANN}(x_k, u_k; \theta^*)$ (with trained weights $\theta^*$) is obtained, it can replace the first-principles or linear model in the NMPC prediction step:

$\hat{x}_{k+1|k} = f_{ANN}(\hat{x}_{k|k}, u_{k|k}; \theta^*)$
$\hat{x}_{k+2|k} = f_{ANN}(\hat{x}_{k+1|k}, u_{k+1|k}; \theta^*)$
... and so on for $N_p$ steps.

The rest of the NMPC formulation (objective function, constraints, NLP solution) remains conceptually the same, but the model evaluations now involve passing data through the trained ANN.

**Challenges for ANN-MPC:**

1.  **Gradient Computation for the NLP Solver:**
    *   NMPC solvers (like SQP or IPMs) require gradients of the objective function and constraints with respect to the decision variables $\mathbf{U}_k$. Since the predictions $\hat{x}_{k+j|k}$ (and thus $\hat{y}_{k+j|k}$) depend on $\mathbf{U}_k$ through multiple recursive calls to $f_{ANN}$, these gradients must be computed by "backpropagating through time" or unrolling the prediction sequence and applying the chain rule through each ANN evaluation.
    *   **Automatic Differentiation (AD)** tools (e.g., TensorFlow, PyTorch, CasADi) are essential here. If the ANN was trained in a framework that supports AD, these frameworks can often be used to provide the necessary gradients to the NLP solver.

2.  **Non-convexity and Local Minima:**
    *   The ANN function $f_{ANN}$ is typically highly nonlinear and non-convex. This means the resulting NMPC optimization problem will also be non-convex, and NLP solvers may converge to local minima.

3.  **Computational Cost:**
    *   Evaluating a deep neural network (especially an RNN/LSTM) multiple times for prediction within each iteration of an NLP solver can be computationally intensive, especially if done on a CPU. GPUs can accelerate ANN evaluations.

4.  **Extrapolation and Model Uncertainty:**
    *   ANNs generally interpolate well within the domain of their training data but extrapolate poorly to regions they haven't seen. If the MPC drives the system into such regions, the ANN model's predictions can become unreliable.
    *   Standard ANNs don't inherently provide a measure of their predictive uncertainty, which is problematic for robust MPC. (Bayesian Neural Networks or ensemble methods can provide some uncertainty estimates).

### A.4 Advantages and Disadvantages of ANN-MPC

**Advantages:**

*   **Ability to Model Complex Unknown Nonlinearities:** Can capture dynamics that are difficult or impossible to model from first principles.
*   **Reduced Modeling Effort (Potentially):** If sufficient data is available, it can be faster to train an ANN than to develop a detailed mechanistic model.
*   **Handles "Soft" Phenomena:** Can potentially learn relationships involving difficult-to-quantify factors if they are implicitly represented in the training data.

**Disadvantages:**

*   **"Black-Box" Nature:** Offers little physical insight into the process dynamics. Difficult to interpret *why* the model behaves a certain way.
*   **Data Hungry:** Requires large, representative datasets for training.
*   **Poor Extrapolation:** Predictions can be unreliable outside the training data range.
*   **Computational Burden:** Can be heavy for online NMPC, especially for deep or recurrent networks.
*   **No Inherent Uncertainty Quantification:** Standard ANNs don't provide confidence bounds on their predictions, making robust design harder.
*   **Training Complexity:** Selecting an appropriate architecture and hyperparameters, and avoiding overfitting, requires expertise.
*   **Verification and Validation:** Difficult to formally verify the properties of an ANN model or an ANN-MPC controller.

### A.5 Hybrid Approaches: Combining First-Principles with ANNs

To mitigate some disadvantages of pure ANN models, hybrid modeling is a promising direction:

1.  **ANNs for Error/Residual Modeling:**
    *   Use a first-principles model for the known parts of the dynamics.
    *   Train an ANN to model the residual error between the first-principles model predictions and the actual plant data.
    *   The overall model is $x_{k+1} = f_{FP}(x_k, u_k) + f_{ANN,error}(x_k, u_k)$.
    *   This leverages existing knowledge and uses the ANN to capture unmodeled effects.

2.  **ANNs for Specific Sub-components:**
    *   If a particular part of a larger first-principles model is poorly understood or too complex (e.g., a complex reaction kinetic term), an ANN can be trained to represent just that sub-component.

3.  **Physics-Informed Neural Networks (PINNs):** (Covered in Appendix B)
    *   ANNs whose training incorporates known physical laws (ODEs/PDEs) in the loss function, leading to more physically plausible and data-efficient models.

*Bioreactor Link (ANN-MPC):*
*   ANNs could be trained to model the complex relationship between nutrient concentrations, byproduct levels, and difficult-to-model cell states (e.g., specific productivity, stress levels) from historical batch data.
*   An ANN might learn the intricate influence of raw material variability (if measurable features of the raw material are available) on process outcomes.
*   A hybrid approach might use standard mass balance equations but employ an ANN to model the specific growth rate ($\mu$) or product formation rate ($q_P$) if these are poorly characterized by simple kinetic expressions.

**In summary, ANNs offer a flexible and powerful tool for data-driven modeling within an MPC framework, especially when first-principles models are inadequate or unavailable. However, their use comes with challenges related to data requirements, computational cost, interpretability, and robustness. Careful training, validation, and integration with AD-capable NLP solvers are key to successful ANN-MPC implementation. Hybrid approaches often provide a good balance by combining the strengths of mechanistic modeling with the learning capabilities of ANNs.**

---