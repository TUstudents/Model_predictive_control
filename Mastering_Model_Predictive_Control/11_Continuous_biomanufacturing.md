## Chapter 11: Case Study: MPC for Continuous Biomanufacturing – Steering Perfusion Systems to Steady Productivity

*"Continuity gives us roots; change gives us branches, letting us stretch and grow and reach new heights." - Pauline R. Kezer*

While fed-batch processes are prevalent, continuous biomanufacturing is gaining significant traction, especially for producing labile proteins or when seeking higher volumetric productivity and product consistency. Perfusion bioreactor systems are a key enabling technology for continuous upstream processing. Unlike the finite endpoint of a fed-batch culture, perfusion systems aim for prolonged, stable operation at a highly productive steady state. This chapter explores the application of Model Predictive Control (MPC) to manage the unique dynamics of perfusion bioreactors, focusing on maintaining high viable cell densities and consistent product quality under continuous operation. Kezer's quote aptly reflects the goal: establishing a continuous, rooted steady state ($continuity$) while being able to adapt to changes and optimize ($branches$).

### 11.1 Introduction to Continuous Biomanufacturing and Perfusion Systems

**Why Continuous?**

*   **Higher Volumetric Productivity:** By continuously feeding fresh media and removing waste/product, cells can be maintained at high densities for extended periods, leading to more product per reactor volume per time.
*   **Improved Product Quality and Consistency:** Operating at a steady state can lead to more consistent critical quality attributes (CQAs) of the product compared to the dynamic environment of a batch/fed-batch process.
*   **Smaller Facility Footprint:** Higher productivity can mean smaller bioreactors and potentially reduced capital expenditure.
*   **Integration with Continuous Downstream Processing:** Enables fully continuous "end-to-end" manufacturing.

**Perfusion Bioreactors:**

*   **Core Principle:** Cells are retained within the bioreactor while spent media (containing product and waste) is continuously removed and replaced with fresh media.
*   **Cell Retention Devices:**
    *   **Alternating Tangential Flow (ATF) Filtration:** Uses a diaphragm pump to create alternating tangential flow across a hollow fiber membrane, minimizing fouling.
    *   **Tangential Flow Filtration (TFF):** Similar to ATF but typically uses a different pumping mechanism.
    *   **Centrifugation or Acoustic Separation:** Other methods for retaining cells.
*   **Key Operational Parameters:**
    *   **Perfusion Rate (or Cell-Specific Perfusion Rate - CSPR):** The rate at which media is exchanged, often normalized per cell (e.g., reactor volumes/day or pL/cell/day).
    *   **Bleed Rate:** A portion of the cell-containing culture is deliberately removed (bled) from the bioreactor to control cell density and maintain a healthy cell population by removing older or dead cells. This prevents the VCD from growing indefinitely.

**(Figure 11.1: Schematic of a perfusion bioreactor with an ATF/TFF cell retention device, media inflow, perfusate outflow (product + waste), and cell bleed.)**

**Challenges in Perfusion Control:**

*   Maintaining a target viable cell density (VCD) in the face of changing growth rates or cell death.
*   Preventing filter fouling in ATF/TFF systems.
*   Ensuring consistent nutrient supply and waste removal to maintain optimal cell physiology and product quality.
*   Long-term operational stability (weeks to months).

### 11.2 Control Objectives in Perfusion Systems

The control objectives for perfusion systems differ significantly from fed-batch:

1.  **Maintain Target Viable Cell Density (VCD):** Operate at a specific high VCD that is optimal for productivity and process stability (e.g., $50-150 \times 10^6$ cells/mL).
2.  **Achieve and Maintain Steady-State Operation:** Ensure that cell concentrations, metabolite levels, and product formation rates are stable over time.
3.  **Consistent Product Quality:** Minimize variability in CQAs by maintaining a consistent cellular environment.
4.  **Maximize Volumetric Productivity:** Produce the maximum amount of product per reactor volume per day.
5.  **Minimize Media Consumption:** Media is a significant cost factor, so optimizing its usage (e.g., by adjusting CSPR) is important.
6.  **Manage Filter Performance:** For ATF/TFF, operate in a way that minimizes fouling and extends filter life.

MPC can help manage the interplay between these objectives by dynamically adjusting perfusion and bleed rates.

### 11.3 Process Modeling for Perfusion Systems

The dynamic model for a perfusion system shares similarities with fed-batch models but includes terms for continuous perfusate removal and cell bleed.

**Key State Variables:** $X_v$ (viable cells), $X_t$ (total cells), $S_i$ (substrates), $P_j$ (products), $L_k$ (byproducts), $V$ (volume, often assumed constant or tightly controlled).

**Model Equations (assuming constant volume $V$ for simplicity, $F_{in} = F_{perfusate} + F_{bleed}$):**

Let $D_p = F_{perfusate}/V$ be the perfusion dilution rate.
Let $D_b = F_{bleed}/V$ be the bleed dilution rate.
The total dilution rate is $D = D_p + D_b$.

1.  **Viable Cell Density ($X_v$):**
    $\frac{dX_v}{dt} = (\mu - \mu_d)X_v - D_b X_v$
    *   Note: $X_v$ is *not* diluted by $D_p$ because cells are retained. Only the bleed stream $D_b$ removes viable cells. If cell retention is imperfect (sieving coefficient $\sigma_X \neq 0$), then a term $-(1-\sigma_X)D_p X_v$ might be added. Assuming perfect retention here.

2.  **Total Cell Density ($X_t$):** (if modeling dead cells explicitly $X_d = X_t - X_v$)
    $\frac{dX_t}{dt} = \mu X_v - D_b X_t$

3.  **Substrate ($S$):** (e.g., glucose)
    $\frac{dS}{dt} = D(S_{feed} - S) - q_S X_v$
    *   $S_{feed}$ is the substrate concentration in the fresh media.
    *   $q_S = (\mu/Y_{X/S}) + m_S (+ \text{terms for product formation})$.
    *   The substrate is diluted by the total media exchange rate $D$.

4.  **Product ($P$):** (e.g., mAb)
    $\frac{dP}{dt} = q_P X_v - D P$
    *   The product is removed in the perfusate (and bleed if present), so it's diluted by $D$. If the product is partially retained by the filter (sieving coefficient $\sigma_P < 1$), then the dilution term is $-(1-\sigma_P)D_p P - D_b P$. Assuming $\sigma_P=0$ (product freely passes).

5.  **Byproduct ($L$):** (e.g., lactate)
    $\frac{dL}{dt} = q_L X_v - D L$
    *   Similar to product, byproducts are removed with the perfusate.

**Kinetics ($\mu, \mu_d, q_S, q_P, q_L$):** The same nonlinear kinetic expressions as used in fed-batch models (Monod, Haldane, Luedeking-Piret, inhibition terms, etc.) apply here, depending on $S, P, L$, etc.

**Steady-State Analysis:**
At steady state ($\frac{dX_v}{dt}=0, \frac{dS}{dt}=0$, etc.), these equations can be solved to find relationships between operating parameters and steady-state values. For example, at steady state for $X_v$:
$\mu - \mu_d = D_b$
This critical relationship shows that the net growth rate must balance the bleed dilution rate to maintain a constant VCD. This is a key control target.

### 11.4 Manipulated Variables and Key Constraints

**Manipulated Variables (MVs for MPC):**

*   **Perfusion Rate ($F_{perfusate}$ or $D_p$):** Controls nutrient supply and waste removal. Directly impacts substrate and byproduct concentrations.
*   **Bleed Rate ($F_{bleed}$ or $D_b$):** Primarily used to control VCD.
*   (Optionally: Feed media concentrations, if multiple streams or online blending is possible).

**Key Constraints:**

*   **Input Constraints:**
    *   $D_{p,min} \le D_p \le D_{p,max}$
    *   $0 \le D_b \le D_{b,max}$
*   **State/Output Constraints:**
    *   $X_{v,min} \le X_v(t) \le X_{v,max}$ (target operating range for VCD)
    *   $S_{glc,min} \le S_{glc}(t)$ (avoid nutrient depletion)
    *   $L_{lac}(t) \le L_{lac,max}$ (limit byproduct accumulation)
    *   Pressure drop across filter ($\Delta P_{filter}$) $\le \Delta P_{max}$ (to prevent fouling/damage, this might be an output to be controlled indirectly by manipulating $D_p$ or by triggering backwashes).
*   **Operational Constraints:**
    *   $D_b < \mu_{net,max}$ (to avoid washout if bleed is too aggressive and growth cannot keep up).

### 11.5 MPC Formulation for Setpoint Tracking and Disturbance Rejection

For continuous perfusion, MPC is often used to:
1.  Drive the system to a desired high-VCD steady state.
2.  Maintain this steady state in the face of disturbances (e.g., changes in cell growth rate, variations in media quality).
3.  Allow for controlled transitions between different steady states if desired.

**Objective Function (Tracking NMPC):**
The objective is typically to penalize deviations of key variables (like $X_v$ and $S_{glc}$) from their desired steady-state setpoints ($X_{v,sp}, S_{glc,sp}$), and penalize excessive control effort.

$J = \sum_{j=1}^{N_p} \left( ||X_v(k+j|k) - X_{v,sp}||^2_{Q_{Xv}} + ||S_{glc}(k+j|k) - S_{glc,sp}||^2_{Q_S} \right) + \sum_{j=0}^{N_c-1} \left( ||\Delta D_p(k+j|k)||^2_{R_{Dp}} + ||\Delta D_b(k+j|k)||^2_{R_{Db}} \right)$

*   $N_p$ is chosen to be long enough to see the effect of current control actions on the approach to steady state.
*   $N_c$ is the control horizon for $D_p, D_b$.
*   The MPC uses the nonlinear perfusion model for prediction.
*   State estimation (MHE, EKF/UKF) is crucial for providing $X_v(k), S_{glc}(k)$, etc., from online (capacitance, OUR/CER) and infrequent offline measurements.

**Disturbance Rejection:**
The inherent feedback in MPC, combined with good state estimation and potentially disturbance modeling (as in Chapter 5, e.g., estimating an offset in $\mu_{max}$), allows the controller to adjust $D_p$ and $D_b$ to counteract unmeasured disturbances and maintain the target steady state. For instance, if cell growth rate unexpectedly decreases, the MPC would reduce $D_b$ to prevent VCD from dropping. If substrate consumption increases, it would increase $D_p$ (or adjust $S_{feed}$ if possible) to maintain $S_{glc,sp}$.

### 11.6 Potential for Economic MPC in Optimizing Steady-State Operation

While tracking MPC is common, EMPC can also be applied to perfusion systems.
*   **Objective:** Maximize steady-state profit rate, e.g., (Value of product harvested per day) - (Cost of media consumed per day) - (Operational costs per day).
    $L_{econ} = (\text{Productivity } [g/L/day] \cdot \text{Value } [\$/g]) - (D_p \cdot \text{Media Cost } [\$/L])$
*   The EMPC would search for the optimal steady-state VCD, $D_p$, and $D_b$ that maximize this economic objective, subject to all process constraints. This could involve finding a VCD that balances higher cell numbers (more product) with higher media consumption and potentially higher bleed rates.
*   The transition to this economically optimal steady state would also be managed by the EMPC.

### 11.7 Integration with PAT for Real-Time Monitoring and Control

Process Analytical Technology (PAT) plays a vital role in enabling advanced control of perfusion systems.
*   **Online Sensors:** Capacitance probes (for VCD estimation), Raman spectroscopy (for glucose, lactate, product estimation), viable cell counters (e.g., flow cytometry based, if automated for at-line use), off-gas analyzers.
*   **Data Analytics & Soft Sensors:** Models that correlate readily available online data (e.g., capacitance, OUR) to difficult-to-measure states ($X_v$, specific metabolite concentrations) provide crucial real-time inputs to the state estimator and MPC.
*   The MPC acts as the "brain" that interprets these PAT data streams (via state estimation) and makes intelligent control decisions.

### 11.8 Simulation Results: Demonstrating Stability and Performance

A case study would typically demonstrate:

1.  **Attainment of Target Steady State:** Show the MPC driving $X_v, S_{glc}$, etc., from initial conditions to their desired steady-state setpoints.
2.  **Disturbance Rejection:** Introduce simulated disturbances (e.g., a step change in $\mu_{max}$ or $S_{feed}$) and show how the MPC adjusts $D_p$ and $D_b$ to return the system to the setpoints.
3.  **Constraint Handling:** Show that the MPC respects all defined constraints during operation.
4.  **Comparison:** Compare MPC performance against simpler strategies like manual control or PID loops controlling VCD via bleed and glucose via perfusion (which can suffer from interactions and oscillations).

**(Figure 11.2: Example simulation plot showing $X_v, S_{glc}, D_p, D_b$ under MPC control, achieving a steady state and rejecting a disturbance.)**

### 11.9 Discussion: The Future of Continuous Bioprocessing Control

**Benefits of MPC for Perfusion:**
*   Systematic management of multiple interacting variables to achieve and maintain a productive steady state.
*   Enhanced robustness to process disturbances.
*   Ability to explicitly handle critical operational constraints.
*   Potential for economic optimization of steady-state operating conditions.

**Challenges:**
*   Long-term model stability and parameter adaptation are even more critical due to the extended duration of perfusion runs.
*   Sensor drift and reliability over long periods.
*   Complexity of modeling and predicting filter performance and fouling.
*   Validation of such advanced control strategies for GMP manufacturing.

*Feynman Insight:* Controlling a perfusion bioreactor with MPC is like expertly sailing a ship across a vast ocean for months on end (long-term steady state). The captain (MPC) constantly checks the ship's position and heading (state estimation from PAT data), adjusts sails and rudder (perfusion and bleed rates) to account for changing winds and currents (disturbances), all while ensuring the ship stays on course (tracks setpoints) and doesn't run aground or out of supplies (respects constraints), aiming for the most efficient journey (economic optimization).

The application of MPC to continuous perfusion systems is a key enabler for the "biopharma factory of the future," promising more efficient, consistent, and intelligently controlled manufacturing processes. The final chapter will look at the broader practical implementation aspects and future horizons for MPC technology.

---