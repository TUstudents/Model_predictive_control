## Chapter 4: Nonlinear and Bioprocess Modeling for Advanced Control

*"The greatest enemy of knowledge is not ignorance, it is the illusion of knowledge." - Stephen Hawking*

Linear models, as discussed in Chapter 3, are powerful and form the basis of widely successful LMPC applications. However, many real-world systems, particularly in chemical and biological engineering, exhibit significant nonlinear behavior that cannot be adequately captured by a linear approximation, especially over a wide range of operating conditions. Attempting to control such systems with an LMPC based on an "illusion" of linearity can lead to poor performance or even instability. This chapter ventures into the realm of nonlinear modeling, with a special focus on developing models for **bioprocesses**, which are notorious for their complex, nonlinear dynamics. These models will pave the way for Nonlinear Model Predictive Control (NMPC).

### 4.1 Introduction to Nonlinear Systems Modeling: Beyond Straight Lines

Linear systems obey the principle of superposition: the response to a sum of inputs is the sum of the responses to individual inputs. Nonlinear systems do not. Their behavior can be much richer and more complex:

*   **Input-Dependent Dynamics:** The system's response characteristics (e.g., gain, time constants) can change significantly depending on the operating point or the magnitude of the input.
*   **Harmonics and Subharmonics:** A sinusoidal input might produce outputs containing frequencies that are multiples or fractions of the input frequency.
*   **Multiple Steady States:** For the same set of inputs, the system might settle into different stable operating points depending on its history.
*   **Limit Cycles:** Self-sustaining oscillations can occur.
*   **Bifurcations:** Small changes in parameters can lead to sudden, qualitative changes in system behavior.
*   **Chaos:** Seemingly random, unpredictable behavior governed by deterministic equations.

*Bioreactor Relevance:* Bioprocesses are rife with nonlinearities.
*   **Cell growth kinetics** (e.g., Monod, Haldane) are highly nonlinear functions of substrate, product, and biomass concentrations.
*   **Enzyme kinetics** often follow Michaelis-Menten or more complex nonlinear rate laws.
*   **pH and temperature dependencies** of biological rates are typically bell-shaped or exponential, far from linear.
*   Interactions between different species in a mixed culture can be strongly nonlinear.

When these nonlinearities are dominant, a linear model identified around a single operating point will fail to predict system behavior accurately when the system moves away from that point. This necessitates the use of nonlinear models for effective control over broader operating ranges.

### 4.2 First-Principles Nonlinear Modeling for Bioprocesses: Capturing the Biology

For bioprocesses, first-principles models are often preferred if the underlying biological and chemical mechanisms are reasonably well understood. These models are typically based on **mass balances** for key components and **kinetic rate expressions** describing their transformations.

**Common Components of a Bioreactor Model:**

Let $V$ be the volume of the liquid in the bioreactor.
For a fed-batch or continuous bioreactor with feed rate $F_{in}$ and outlet rate $F_{out}$:
$\frac{dV}{dt} = F_{in} - F_{out}$ (If $F_{out}=0$, it's a fed-batch process)

1.  **Biomass ($X$):** Concentration of cells (e.g., g/L or cells/mL).
    $\frac{d(VX)}{dt} = (\mu - k_d)XV - F_{out}X$
    Expanding the left side (product rule): $V\frac{dX}{dt} + X\frac{dV}{dt} = (\mu - k_d)XV - F_{out}X$
    $V\frac{dX}{dt} = (\mu - k_d)XV - F_{out}X - X(F_{in} - F_{out})$
    $\frac{dX}{dt} = (\mu - k_d - \mu_D)X \quad$ where $\mu_D = \frac{F_{in}}{V}$ is the dilution rate by feed.
    If $F_{out}$ is a bleed stream with biomass, this term needs adjustment. Often in fed-batch $F_{out}=0$.
    A common form for fed-batch ($F_{out}=0$):
    $\frac{dX}{dt} = (\mu - k_d)X - \frac{F_{in}}{V}X$
    *   $\mu$: Specific growth rate (1/time), the core of biological kinetics.
    *   $k_d$: Specific death rate or endogenous metabolism rate (1/time).

2.  **Substrate(s) ($S_i$):** Concentration of limiting nutrients (e.g., glucose, glutamine, oxygen).
    For a primary substrate $S$:
    $\frac{dS}{dt} = -\frac{\mu X}{Y_{X/S}} - \frac{q_P X}{Y_{P/S}} - m_S X + \frac{F_{in}}{V}(S_{feed} - S)$
    *   $Y_{X/S}$: Yield of biomass from substrate (g biomass / g substrate).
    *   $q_P$: Specific product formation rate (g product / g biomass / time).
    *   $Y_{P/S}$: Yield of product from substrate (g product / g substrate), relevant if product formation consumes substrate directly beyond growth needs.
    *   $m_S$: Maintenance coefficient (g substrate / g biomass / time), substrate consumed for functions other than growth or product formation.
    *   $S_{feed}$: Substrate concentration in the feed stream.

3.  **Product(s) ($P_j$):** Concentration of desired products (e.g., proteins, antibodies, metabolites).
    For a product $P$:
    $\frac{dP}{dt} = q_P X - k_{deg}P - \frac{F_{in}}{V}P$
    *   $k_{deg}$: Product degradation rate constant (1/time).

**Kinetic Rate Expressions (The Nonlinear Core):**

These empirical or semi-empirical expressions describe how $\mu, q_P, m_S$ depend on concentrations of substrates, products, inhibitors, etc.

*   **Specific Growth Rate ($\mu$):**
    *   **Monod Model (Substrate Limitation):**
        $\mu = \mu_{max} \frac{S}{K_S + S}$
        $S$: Limiting substrate concentration.
        $\mu_{max}$: Maximum specific growth rate.
        $K_S$: Monod constant (substrate affinity constant; concentration at which $\mu = \mu_{max}/2$).
        **(Figure 4.1: Plot of Monod kinetics showing saturation.)**
    *   **Haldane Model (Substrate Inhibition):** For substrates that are inhibitory at high concentrations (e.g., some phenols, alcohols).
        $\mu = \mu_{max} \frac{S}{K_S + S + S^2/K_I}$
        $K_I$: Substrate inhibition constant.
        **(Figure 4.2: Plot of Haldane kinetics showing an optimal substrate concentration.)**
    *   **Multiple Substrate Limitation:** Can be modeled using multiplicative terms or more complex expressions (e.g., $\mu = \mu_{max} (\frac{S_1}{K_{S1}+S_1}) (\frac{S_2}{K_{S2}+S_2})$).
    *   **Product Inhibition:** $\mu = \mu_{max} \frac{S}{K_S + S} \frac{K_{IP}}{K_{IP} + P}$ or similar forms.

*   **Specific Product Formation Rate ($q_P$):**
    *   **Luedeking-Piret Model (Growth-associated and Non-growth-associated):**
        $q_P = \alpha \mu + \beta$
        $\alpha$: Growth-associated product formation coefficient.
        $\beta$: Non-growth-associated product formation coefficient.
        Can be extended to include substrate/product dependencies: $q_P = (\alpha \mu + \beta) \frac{S}{K_{SP} + S} \frac{K_{IPP}}{K_{IPP} + P}$.
    *   Sometimes $q_P$ is directly proportional to $\mu$ ($q_P = Y_{P/X} \mu$).

*   **Influence of Other Factors:**
    *   **Temperature ($T$):** Often modeled using an Arrhenius-type equation or a bell-shaped curve (e.g., Cardinal Temperature Model).
        $\mu(T) = \mu_{opt} \frac{(T-T_{max})(T-T_{min})^2}{(T_{opt}-T_{min})[(T_{opt}-T_{min})(T-T_{opt}) - (T_{opt}-T_{max})(T_{opt}+T_{min}-2T)]}$ (Ratkowsky model variant)
    *   **pH:** Similar bell-shaped dependencies around an optimal pH.
        $\mu(pH) = \mu_{opt} \frac{K_{pH1}}{K_{pH1} + [H^+]} \frac{[H^+]}{K_{pH2} + [H^+]}$ (simplified representation for effects below and above optimum).

**Challenges in First-Principles Bioprocess Modeling:**

*   **Parameter Identifiability:** Bioprocess models often have many parameters ($\mu_{max}, K_S, Y_{X/S}$, etc.). Estimating all of them accurately from limited, noisy experimental data can be very difficult. Parameters might be correlated, or data might not be sufficiently informative to distinguish their individual effects.
*   **Unmodeled Dynamics:** Cells are incredibly complex. Simplified models inevitably neglect some metabolic pathways, regulatory mechanisms, or population heterogeneity.
*   **Biological Variability:** Batch-to-batch variations due to inoculum quality, subtle environmental shifts, or genetic drift can lead to changes in model parameters over time.
*   **Measurement Scarcity:** Key state variables like intracellular metabolite concentrations are often not measurable online. Biomass itself might require infrequent, offline sampling.

*Feynman Insight:* Modeling a cell culture is like trying to write down the rules for a bustling city based on observing only its total population, food intake, and waste output. You can capture the main trends, but the internal complexity is immense. The goal is a model that's good enough to guide our "city planning" (control inputs) for desired outcomes.

### 4.3 Data-Driven Nonlinear Modeling Approaches

When first-principles understanding is lacking, or the resulting models are too complex, data-driven nonlinear modeling techniques can be employed. These treat the system more like a "black box."

1.  **Block-Oriented Models:**
    *   **Hammerstein Model:** A static nonlinearity followed by linear dynamics. $u \rightarrow [Nonlinear block] \rightarrow w \rightarrow [Linear block] \rightarrow y$.
    *   **Wiener Model:** Linear dynamics followed by a static nonlinearity. $u \rightarrow [Linear block] \rightarrow w \rightarrow [Nonlinear block] \rightarrow y$.
    *   **Hammerstein-Wiener Model:** A linear block sandwiched between two static nonlinearities.
    *   These can capture some forms of nonlinearity while leveraging linear system identification tools for the dynamic part.

2.  **Volterra Series Models:**
    *   A generalization of the convolution integral for linear systems, essentially a "nonlinear impulse response." Can become very complex with many terms (kernels) to identify.

3.  **Artificial Neural Networks (ANNs):** (Detailed in Appendix A)
    *   Universal approximators capable of learning complex nonlinear input-output mappings from data.
    *   Common types: Feedforward Neural Networks (FNNs), Recurrent Neural Networks (RNNs) like LSTMs are good for dynamic systems.
    *   **Pros:** Highly flexible, can capture unknown nonlinearities.
    *   **Cons:** Require large amounts of training data, can be prone to overfitting, "black-box" nature offers little physical insight, extrapolation is often poor, integration into NMPC solvers can be complex (gradients needed).

4.  **Gaussian Processes (GPs):**
    *   A non-parametric Bayesian approach that places a prior over functions.
    *   Provides predictions along with uncertainty estimates (confidence intervals).
    *   **Pros:** Good for small datasets, provides uncertainty quantification.
    *   **Cons:** Can be computationally intensive for large datasets and high dimensions.

5.  **Physics-Informed Neural Networks (PINNs):** (Detailed in Appendix B)
    *   A hybrid approach where ANNs are trained to satisfy both data and known physical laws (e.g., ODEs/PDEs that govern the system). The physics residuals are included in the loss function.
    *   **Pros:** Can improve generalization, require less data than pure ANNs, produce more physically plausible models.
    *   **Cons:** Requires knowledge of the governing equations (even if parameters are unknown), training can be more complex.

### 4.4 Model Reduction and Simplification for NMPC

Even if a detailed first-principles nonlinear model is available, it might be too computationally expensive to solve repeatedly within an NMPC loop. Model reduction aims to create a simpler model that retains the essential dynamics for control.

*   **Sensitivity Analysis:** Identify parameters or states that have little impact on the outputs of interest and can potentially be fixed or removed.
*   **Time-Scale Separation:** If some dynamics are much faster than others, the fast dynamics can sometimes be approximated as being at quasi-steady state.
*   **Proper Orthogonal Decomposition (POD) / Karhunen-Loève Transform:** A technique for finding a low-dimensional basis that captures most of the variance in high-dimensional data or model states. Can be used to reduce the order of discretized PDEs or large ODE systems.
*   **Balanced Truncation / Realization:** Primarily for linear systems, but nonlinear balancing concepts exist.

The choice of modeling approach and the level of detail depend on:
*   Available prior knowledge of the system.
*   Quantity and quality of available data.
*   The degree of nonlinearity.
*   Computational resources available for online MPC.
*   The specific control objectives.

### 4.5 Example: Step-by-Step Development of a Fed-Batch Bioreactor Model

Let's outline the construction of a typical fed-batch model for producing a recombinant protein using mammalian cells (e.g., CHO cells).

**States:**
*   $X_v$: Viable cell density (cells/mL)
*   $S_{glc}$: Glucose concentration (g/L)
*   $S_{gln}$: Glutamine concentration (g/L)
*   $P_{mab}$: Monoclonal antibody (product) concentration (g/L)
*   $L_{lac}$: Lactate concentration (byproduct) (g/L)
*   $A_{amm}$: Ammonia concentration (byproduct) (g/L)
*   $V$: Volume (L)

**Inputs (Manipulated Variables for MPC):**
*   $F_{glc}$: Feed rate of concentrated glucose solution (L/hr)
*   $F_{gln}$: Feed rate of concentrated glutamine solution (L/hr)
    (Often combined into a single nutrient feed $F_{feed}$ with known $S_{glc,feed}, S_{gln,feed}$)

**Model Equations (Conceptual - specific forms for $\mu, q$ vary widely):**

1.  **Volume:**
    $\frac{dV}{dt} = F_{glc} + F_{gln}$ (assuming separate feeds)

2.  **Viable Cell Density:**
    $\frac{dX_v}{dt} = (\mu - \mu_d)X_v - \frac{F_{glc}+F_{gln}}{V}X_v$
    *   $\mu = f_1(S_{glc}, S_{gln}, L_{lac}, A_{amm}, \text{other factors like pH, T, DO})$
        (e.g., $\mu = \mu_{max,ref} \frac{S_{glc}}{K_{S,glc}+S_{glc}} \frac{S_{gln}}{K_{S,gln}+S_{gln}} \frac{K_{I,lac}}{K_{I,lac}+L_{lac}} \frac{K_{I,amm}}{K_{I,amm}+A_{amm}}$)
    *   $\mu_d = f_2(L_{lac}, A_{amm}, \text{nutrient depletion})$ (specific death rate)

3.  **Glucose:**
    $\frac{dS_{glc}}{dt} = -q_{S,glc}X_v + \frac{F_{glc}}{V}(S_{glc,feed} - S_{glc}) - \frac{F_{gln}}{V}S_{glc}$
    *   $q_{S,glc} = \frac{\mu}{Y_{X/glc}} + m_{S,glc} (+ \frac{q_{P,mab}}{Y_{P/glc}} \text{ if product formation uses glucose})$
    *   Part of $q_{S,glc}$ might also go to lactate: $q_{S,glc \rightarrow L_{lac}}$

4.  **Glutamine:**
    $\frac{dS_{gln}}{dt} = -q_{S,gln}X_v - k_{deg,gln}S_{gln} + \frac{F_{gln}}{V}(S_{gln,feed} - S_{gln}) - \frac{F_{glc}}{V}S_{gln}$
    *   $q_{S,gln} = \frac{\mu}{Y_{X/gln}} + m_{S,gln}$
    *   $k_{deg,gln}$: Chemical degradation rate of glutamine.

5.  **Lactate:**
    $\frac{dL_{lac}}{dt} = q_{L,lac}X_v - q_{C,lac}X_v - \frac{F_{glc}+F_{gln}}{V}L_{lac}$
    *   $q_{L,lac} = Y_{L/glc} \cdot q_{S,glc \rightarrow L_{lac}}$ (lactate production rate from glucose)
    *   $q_{C,lac}$: Specific lactate consumption rate (cells can switch to consuming lactate under certain conditions).

6.  **Ammonia:**
    $\frac{dA_{amm}}{dt} = q_{A,amm}X_v - \frac{F_{glc}+F_{gln}}{V}A_{amm}$
    *   $q_{A,amm} = Y_{A/gln} \cdot q_{S,gln}$ (ammonia production primarily from glutamine metabolism).

7.  **Product (mAb):**
    $\frac{dP_{mab}}{dt} = q_{P,mab}X_v - \frac{F_{glc}+F_{gln}}{V}P_{mab}$
    *   $q_{P,mab} = f_3(\mu, S_{glc}, S_{gln}, \text{other factors})$ (e.g., Luedeking-Piret type, possibly linked to viable cell integral or specific metabolic states).

This system of coupled, nonlinear ODEs, once its many parameters ($K_S, Y, \mu_{max}$, etc.) are estimated from experimental data (often a significant undertaking involving dedicated experiments and parameter estimation algorithms), can then be used within an NMPC framework. The NMPC would try to find optimal feeding profiles $F_{glc}(t)$ and $F_{gln}(t)$ to, for instance, maximize $P_{mab}(t_f)$ at the end of the batch $t_f$, subject to constraints on feed rates and metabolite concentrations.

**The models we build, whether linear or nonlinear, are the foundation upon which MPC operates. Their fidelity directly translates to control performance. Having explored modeling, the next chapter will focus on how MPC utilizes these models in conjunction with real-time measurements through state estimation, and how it handles inevitable disturbances.**

---