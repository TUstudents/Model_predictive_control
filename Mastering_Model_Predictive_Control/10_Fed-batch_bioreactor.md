## Chapter 10: Case Study: NMPC for Fed-Batch Bioreactor Optimization – Maximizing Productivity

*"The goal of all life is to have a good time." - Albert Einstein (in a more whimsical mood)*
*"The goal of (many) bioreactor operations is to maximize product, safely and efficiently." - A Bioprocess Engineer*

While Einstein's sentiment is charming, the bioprocess engineer's pragmatic goal often takes precedence in industrial settings. Fed-batch cultures are workhorses in biomanufacturing, particularly for producing therapeutic proteins like monoclonal antibodies (mAbs). Optimizing these processes – maximizing the final product titer, ensuring consistent product quality, and doing so efficiently – is a complex, multivariable challenge. This chapter presents a case study on applying Nonlinear Model Predictive Control (NMPC), potentially in an Economic MPC (EMPC) framework, to optimize a fed-batch bioreactor, illustrating how MPC can navigate the intricate biological landscape to achieve these demanding objectives.

### 10.1 Process Description: Mammalian Cell Culture for Monoclonal Antibody (mAb) Production

*   **The Organism:** Typically, Chinese Hamster Ovary (CHO) cells, genetically engineered to produce a specific mAb. These cells are shear-sensitive and have complex nutritional requirements.
*   **The Bioreactor:** A stirred-tank bioreactor, ranging from bench-scale (liters) to production-scale (thousands of liters). Equipped with sensors for online monitoring of temperature, pH, Dissolved Oxygen (DO), and agitation.
*   **The Mode of Operation: Fed-Batch**
    1.  **Batch Phase:** Cells are inoculated into an initial volume of basal medium. They grow, consuming initial nutrients.
    2.  **Fed-Batch Phase:** Concentrated nutrient feeds (e.g., glucose, amino acids, lipids, vitamins) are added to the bioreactor over time to sustain cell growth, maintain viability, and promote product formation, while avoiding excessive accumulation of byproducts or detrimental osmolality shifts. The volume increases as feed is added.
    3.  **Harvest:** The process is terminated when product titer peaks, viability drops significantly, or a maximum batch time is reached. The product is then harvested from the culture broth for downstream purification.
*   **Key Challenges:**
    *   Preventing nutrient depletion (especially key substrates like glucose and glutamine).
    *   Avoiding accumulation of inhibitory byproducts (e.g., lactate, ammonia).
    *   Maintaining physiological parameters (pH, T, DO) within optimal ranges.
    *   The optimal feeding strategy is often non-intuitive and can vary significantly depending on the cell line and specific product.
    *   Achieving batch-to-batch consistency.

**(Figure 10.1: Schematic of a fed-batch bioreactor with nutrient feeds and key monitored variables.)**

### 10.2 Control Objectives: Beyond Simple Setpoint Tracking

While maintaining pH, T, and DO at fixed setpoints is important (often handled by local PID loops or basic logic), the overarching objectives for the feeding strategy are more complex:

1.  **Maximize Final Product Titer ($P_{mab}(t_f)$):** This is often the primary economic driver.
2.  **Maximize Viable Cell Density (VCD) or Integral of Viable Cell Density (IVCD):** High VCD is usually correlated with high product formation, though productivity can decouple from growth, especially in later phases. IVCD ($\int X_v dt$) is often a good indicator of total production capacity.
3.  **Maintain Critical Quality Attributes (CQAs):** The product must meet quality specifications (e.g., glycosylation patterns for mAbs). Feeding strategies can influence CQAs. This might be translated into constraints on certain metabolite profiles or process conditions.
4.  **Minimize Batch Time ($t_f$):** Shorter batches increase facility throughput, but not at the expense of significantly lower titer.
5.  **Minimize Resource Consumption:** Reduce the use of expensive media components.

These objectives are often conflicting. For instance, very aggressive feeding might boost VCD quickly but lead to byproduct accumulation that harms cells or product quality later. NMPC/EMPC provides a framework to manage these trade-offs.

### 10.3 Detailed Model Development (Illustrative - Cell Line Specific)

A suitable NMPC requires a dynamic model (as discussed in Chapter 4). For CHO cell mAb production, a common model structure includes states for:

*   $V$: Volume
*   $X_v$: Viable cell density
*   $X_t$: Total cell density (or $X_d$: dead cell density, $X_t = X_v + X_d$)
*   $S_{glc}$: Glucose concentration
*   $S_{gln}$: Glutamine concentration
*   $P_{mab}$: mAb concentration
*   $L_{lac}$: Lactate concentration
*   $A_{amm}$: Ammonia concentration
*   (Optionally: other amino acids, lipids, osmolality, product quality attributes if models exist)

**Key Kinetic Expressions (examples):**

*   **Specific Growth Rate ($\mu$):**
    $\mu = \mu_{max} \cdot f_{glc}(S_{glc}) \cdot f_{gln}(S_{gln}) \cdot f_{inh,lac}(L_{lac}) \cdot f_{inh,amm}(A_{amm})$
    Where $f(\cdot)$ are Monod-type terms for substrates and inhibition terms (e.g., non-competitive) for byproducts.
    $\mu_{max}$ might itself be a function of cell age or other factors.
*   **Specific Death Rate ($\mu_d$):**
    $\mu_d = k_{d,base} + k_{d,lac}L_{lac} + k_{d,amm}A_{amm} + k_{d,shear}(\text{if agitation changes})$
    Often also increases with nutrient depletion or accumulation of other toxins.
*   **Specific Glucose Consumption Rate ($q_{glc}$):**
    $q_{glc} = \frac{\mu}{Y_{X/glc}} + m_{glc} (+ \frac{q_{P,mab}}{Y_{P/glc}} \text{ if product formation requires glucose})$
*   **Specific Glutamine Consumption Rate ($q_{gln}$):**
    $q_{gln} = \frac{\mu}{Y_{X/gln}} + m_{gln}$ (also produces ammonia)
*   **Specific Lactate Production/Consumption Rate ($q_{lac}$):**
    Cells can produce lactate (Warburg effect) even with oxygen, $q_{L,prod} = Y_{L/glc} \cdot q_{glc,glycolytic}$.
    Under glucose limitation, they might switch to consuming lactate, $q_{L,cons} = q_{L,cons,max} \frac{L_{lac}}{K_{L,cons}+L_{lac}} \frac{K_{I,glc,Lcons}}{K_{I,glc,Lcons}+S_{glc}}$.
*   **Specific Ammonia Production Rate ($q_{amm}$):**
    Mainly from glutamine metabolism, $q_{amm} = Y_{A/gln} \cdot q_{gln}$.
*   **Specific mAb Production Rate ($q_{P,mab}$):**
    Often modeled as non-growth associated or a complex function of cell specific productivity which might change over the culture duration: $q_{P,mab} = \alpha \mu + \beta$, or $q_{P,mab} = Q_P \cdot (\text{terms depending on nutrient availability, stress, etc.})$
    $Q_P$ (cell-specific productivity) might be constant or vary.

**Parameter Estimation:**
This is a critical and time-consuming step. Parameters ($Y_{X/S}, \mu_{max}, K_S, K_I$, etc.) are estimated by fitting the model to experimental data from multiple fed-batch runs with varying feeding profiles. This often involves solving a large-scale optimization problem itself (minimizing sum of squared errors between model predictions and experimental data).

### 10.4 Manipulated Variables and Process Constraints

**Manipulated Variables (MVs for NMPC):**
Typically, the flow rates of one or more concentrated feed streams:
*   $F_1(t)$: Flow rate of main nutrient feed (containing glucose, amino acids, etc.)
*   $F_2(t)$: Flow rate of a glucose-only feed (for independent glucose control)
*   (Less commonly for NMPC feeding: temperature shifts, pH shifts, though these can be MVs if their profiles are to be optimized).

**Process Constraints:**

*   **Input Constraints:**
    *   $0 \le F_i(t) \le F_{i,max}$ (pump limits)
    *   $\Delta F_{i,min} \le \Delta F_i(t) \le \Delta F_{i,max}$ (rate of change of feed)
*   **State/Output Constraints (often "soft" constraints or heavily penalized in cost function):**
    *   $S_{glc,min} \le S_{glc}(t) \le S_{glc,max}$ (e.g., 0.5 g/L to 5 g/L)
    *   $S_{gln,min} \le S_{gln}(t)$
    *   $L_{lac}(t) \le L_{lac,max}$
    *   $A_{amm}(t) \le A_{amm,max}$
    *   Osmolality$(t) \le Osm_{max}$
*   **Terminal Constraints:**
    *   $V(t_f) \le V_{max}$ (maximum reactor working volume)

### 10.5 State and Parameter Estimation Strategy

As discussed in Chapter 5, key states are not measured online.

*   **Online Measurements:** Temperature, pH, DO, volume (from load cells or feed integration), online capacitance (correlates with VCD), off-gas analysis (OUR, CER – correlate with metabolic activity).
*   **Offline Measurements (Infrequent, Delayed):** VCD, viability, $S_{glc}, S_{gln}, L_{lac}, A_{amm}, P_{mab}$. Samples taken perhaps once or twice a day.
*   **Estimator Choice:**
    *   **Moving Horizon Estimator (MHE)** is well-suited. It can:
        *   Naturally incorporate infrequent, delayed offline measurements.
        *   Use the same nonlinear model as the NMPC.
        *   Enforce state constraints (e.g., concentrations > 0).
        *   Potentially perform joint state and parameter estimation (e.g., adapting key yield or kinetic parameters if they drift).
    *   **Extended/Unscented Kalman Filters (EKF/UKF)** can also be used, often with "data reconciliation" steps when offline data arrives. Soft sensors based on online data can provide intermediate "pseudo-measurements" to the EKF/UKF between lab samples.

**(Figure 10.2: Diagram showing online/offline data flow into an MHE, which provides state estimates to the NMPC.)**

### 10.6 NMPC/EMPC Formulation

The NMPC controller, at each sampling interval (e.g., every 1-4 hours, or more frequently if online sensors drive faster decisions), solves an NLP.

**Decision Variables (for the NLP):**
The sequence of feed rates over the control horizon $N_c$: $\mathbf{F}(k) = [F_1(k|k), \dots, F_1(k+N_c-1|k), F_2(k|k), \dots, F_2(k+N_c-1|k)]^T$.

**Objective Function:**

1.  **Tracking NMPC (if target profiles for $S_{glc}$, $X_v$ are known):**
    $J = \sum_{j=1}^{N_p} ||S_{glc}(k+j|k) - S_{glc,sp}(k+j)||^2_{Q_S} + \sum_{j=1}^{N_p} ||X_v(k+j|k) - X_{v,sp}(k+j)||^2_{Q_X} + \sum_{j=0}^{N_c-1} ||\Delta F(k+j|k)||^2_R$
    (Plus penalties for violating soft constraints on $L_{lac}, A_{amm}$, etc.)

2.  **Economic NMPC (EMPC – more common for global optimization):**
    The goal is to maximize profit or a surrogate like final product titer over the remaining batch duration. The prediction horizon $N_p$ often extends to the anticipated end of the batch $t_f$.
    *   Objective: Minimize $-P_{mab}(t_f|k)$ (maximize final product)
    *   Or: Minimize $\left( \text{Cost of feeds used over } [k, t_f] - \text{Value of } P_{mab}(t_f|k) \right)$
    *   Stage cost $L_{econ,j}$ could represent cost of feed used in interval $j$. Terminal cost $\Phi_{econ}$ would be $-Value \cdot P_{mab}(t_f|k)$.
    *   This allows the NMPC to make dynamic trade-offs. For instance, it might choose to slightly underfeed glucose early on if the model predicts this leads to a more favorable metabolic state for higher productivity later, even if it means slightly slower initial growth.

**NLP Solver and Discretization:**
*   A simultaneous approach (e.g., direct collocation) is common for these potentially stiff and highly nonlinear ODE systems.
*   NLP solvers like IPOPT or SNOPT are typically used, interfaced via tools like CasADi or GEKKO.

### 10.7 Implementation Details and Practical Considerations

*   **Sampling and Control Interval:** State estimation might run continuously using online data, with updates when offline data arrives. The NMPC optimization might be solved less frequently (e.g., every few hours) or when significant deviations from predicted trajectories occur.
*   **Horizon Lengths ($N_p, N_c$):**
    *   $N_p$ for EMPC often covers the remaining batch duration (e.g., 5-10 days, discretized into many steps).
    *   $N_c$ is usually shorter (e.g., 12-24 hours, discretized) to reduce computational load. Feed rates are held constant or follow a simple profile beyond $N_c$.
*   **Initialization of NLP:** Crucial for convergence. The previous optimal feed profile, shifted in time, is a good initial guess.
*   **Robustness:**
    *   The model parameters have uncertainty. Sensitivity analysis can show which parameters are most critical.
    *   Worst-case scenarios or scenario-based NMPC could be considered if computational power allows.
    *   Constraint back-off based on state estimation uncertainty.
*   **Model Maintenance:** The model may need recalibration as the cell line ages or process conditions change. This is a significant lifecycle management activity.

### 10.8 Simulation Results and Comparison

A typical case study would:
1.  Show the performance of the NMPC/EMPC in simulation using the developed model.
2.  Compare its performance (final titer, byproduct levels, batch time) against:
    *   Traditional, pre-defined feeding profiles (e.g., based on historical data or operator experience).
    *   Simpler control strategies (e.g., PID control of glucose concentration by manipulating feed rate).
3.  Demonstrate how the NMPC respects constraints (e.g., keeps lactate below a threshold).
4.  Illustrate the dynamic nature of the optimized feeding profiles, often non-intuitive. For example, the NMPC might choose to allow a controlled period of slight nutrient limitation if its model predicts this will trigger a more productive metabolic state later.

**(Figure 10.3: Example simulation plot showing trajectories of $X_v, S_{glc}, P_{mab}, L_{lac}$ and the optimized feed rate $F(t)$ under NMPC vs. a standard strategy.)**

The results often show significant improvements in final product titer (e.g., 10-30% or more) and better consistency when using NMPC/EMPC compared to conventional approaches, due to its ability to proactively manage multiple interacting variables and constraints based on a predictive model.

### 10.9 Discussion: Benefits, Challenges, and Future Outlook

**Benefits:**
*   Systematic optimization of complex, multivariable objectives.
*   Proactive constraint handling, allowing operation closer to optimal but safe limits.
*   Potential for significant improvements in yield, productivity, and consistency.
*   Adaptability to disturbances if combined with good state/parameter estimation.

**Challenges:**
*   High development effort for model building, parameter estimation, and validation.
*   Computational demands of online NMPC solution.
*   Robustness to significant model uncertainty or unmodeled biological phenomena.
*   Regulatory acceptance (requires thorough validation and documentation, fitting into PAT frameworks).

*Feynman Insight Revisited:* The NMPC for a fed-batch bioreactor is like an exceptionally skilled chef constantly tasting (measuring/estimating), understanding the intricate chemistry of cooking (the model), and adjusting heat and ingredients (feed rates) over hours to create a perfect dish (maximize product), while ensuring nothing burns or boils over (constraints). It’s dynamic, predictive, and highly adaptive orchestration.

This case study demonstrates the immense potential of NMPC to transform bioprocess operation. While challenging to implement, the rewards in terms of process performance and understanding can be substantial. The next case study will explore MPC for continuous biomanufacturing, which presents a different set of objectives and challenges.

---