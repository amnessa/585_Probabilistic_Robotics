This homework explores the two fundamental algorithms for state estimation discussed in *Probabilistic Robotics* (Thrun et al.) and your lecture slides: **Gaussian Filters (Kalman Filters)** and **Non-Parametric Filters (Particle Filters)**.

We are dealing with a classic 1D tracking problem: estimating the position and velocity of a moving object ("car") given noisy dynamics and noisy position measurements.

Here is the breakdown of the questions, the theoretical context from your course materials, and the implementation plan.

### **Question 1: Kalman Filter - Prediction**
This question focuses on the **Prediction Step** of the Bayes Filter using a Linear Gaussian model (Chapter 3.2 of the textbook).

* **Theoretical Concept:** You are modeling a continuous state space with a unimodal Gaussian belief. Since the system is linear (linear dynamics) and noise is Gaussian, the Kalman Filter (KF) is the optimal estimator.
* **The Physics:**
    * **State ($x_t$):** The question asks for a minimal state vector to respect the Markov assumption. Since acceleration affects velocity, and velocity affects position, we need both Position ($p$) and Velocity ($v$) in our state to predict the future. State vector $\mathbf{x} = [p, v]^T$.
    * **Motion Model:** $x_t = A_t x_{t-1} + B_t u_t + \epsilon_t$.
    * Since acceleration is "random" (noise), control input $u_t$ is effectively 0, and the acceleration acts as the process noise $\epsilon_t$.
    * **Process Noise Covariance ($R$):** The random acceleration propagates into position and velocity.
        * Position shift due to accel $a$: $\frac{1}{2} a \Delta t^2$
        * Velocity shift due to accel $a$: $a \Delta t$
        * With $\Delta t = 1$ and $\sigma_a^2=1$, the covariance matrix $R$ captures the correlation between position and velocity errors introduced by this random acceleration.


### **Question 2: Kalman Filter - Measurement Update**
This question focuses on the **Correction Step** (Chapter 3.2.2).

* **Theoretical Concept:** We observe the position ($z$). We compare the *actual* measurement with the *predicted* measurement to calculate the **innovation**. The **Kalman Gain ($K$)** determines how much we trust the measurement versus our prediction.
* **Measurement Model:** $z_t = C_t x_t + \delta_t$.
    * Matrix $C$ extracts the position from the state vector.
    * $Q = 10$ is the measurement noise variance.
* **Uncertainty Ellipses:** You need to visualize the covariance matrix $\Sigma$. The eigenvalues and eigenvectors of $\Sigma$ define the axes and orientation of the ellipse.

### **Question 3: Particle Filter (Monte Carlo Localization)**
This question asks you to solve the exact same problem using a non-parametric approach (Chapter 4.3, Table 4.3).

* **Theoretical Concept:** Instead of a mean vector and covariance matrix, we represent the posterior belief as a set of $N$ particles.
* **Mechanism:**
    1.  **Sampling (Motion):** Move every particle according to the physics (random acceleration).
    2.  **Weighting (Measurement):** Assign weights to particles based on how likely the measurement $z$ is given that particle's position. (Likelihood is a Gaussian PDF centered at the particle's position).
    3.  **Resampling:** Select particles with high weights to survive to the next generation (using Low Variance Sampling, Table 4.4).
* **Comparison:** You will compare the analytical covariance (KF) with the sample covariance of the particle cloud.


---

### **Implementation Plan**

I will generate three Python files. You can run these directly to generate the plots and answers for your homework.

1.  `q1_sol.py`: Sets up the Kalman Filter class, defines matrices A and R, simulates ground truth trajectories, and performs the prediction loop.
2.  `q2_sol.py`: Extends Q1 to include the measurement update ($C$, $Q$, Kalman Gain calculations) and plots uncertainty ellipses updating over time.
3.  `q3_sol.py`: Implements the Particle Filter with `sample_motion_model`, `measurement_model`, and `low_variance_sampler`. It compares the PF results to the KF results.


### **1. Explanation of the Questions**

* **Q1: Kalman Filter Prediction (Linear Gaussian Systems)**
    This question establishes the baseline dynamics of your robot. You are dealing with a **Linear Gaussian System**. As discussed in *Probabilistic Robotics* (Chapter 3.2), when the state transition function is linear and the noise is Gaussian, the optimal estimator is the Discrete Kalman Filter.
    * **Minimal State:** Since acceleration ($\ddot{x}$) is the noise source, your system state must track Position ($x$) and Velocity ($\dot{x}$). Without velocity, you cannot account for the inertial effects of the acceleration noise over time.
    * **Prediction Step:** This involves projecting the state forward. The uncertainty (covariance $\Sigma$) grows during this step because we are adding process noise ($R$) and not gathering new information. You'll see the error ellipses grow and tilt. The "tilt" represents the correlation—velocity errors naturally propagate into position errors over time.

* **Q2: Kalman Filter Correction (Measurement Update)**
    Here, you introduce a sensor. The sensor measures position ($z=x$) with Gaussian noise.
    * **Correction Step:** This reduces uncertainty. The Kalman Gain ($K$) computes the optimal weighted average between your noisy prediction from Q1 and this new noisy measurement.
    * **Ellipses:** You will see the uncertainty ellipses shrink immediately after an update. The orientation of the ellipse will also shift, reflecting how knowledge of position corrects our estimate of velocity indirectly.

* **Q3: Particle Filter (Non-Parametric)**
    This asks you to solve the exact same problem using a **Particle Filter (MCL)** (Chapter 4.3).
    * **Why:** Kalman Filters are restricted to unimodal Gaussian beliefs. Particle filters can represent arbitrary distributions. While this problem *is* Gaussian (and thus KF is optimal), implementing a PF here allows you to benchmark it against the optimal solution.
    * **Resampling:** You will implement **Low Variance Sampling** (Table 4.4 in the book), which is a crucial algorithm for preventing particle depletion while maintaining efficiency ($O(M)$ complexity).

### **2. Implementation Details**

The Python files I have generated (`q1_sol.py`, `q2_sol.py`, `q3_sol.py`) are complete implementations. Here is what they do:

1.  **Covariance Matrices:** I calculated the process noise matrix $R$ derived from the physics of constant acceleration:
    $R = \sigma_{acc}^2 \begin{bmatrix} \frac{1}{4}\Delta t^4 & \frac{1}{2}\Delta t^3 \\ \frac{1}{2}\Delta t^3 & \Delta t^2 \end{bmatrix}$
2.  **Visualization:** The code uses `matplotlib.patches.Ellipse` to draw the $1\sigma$ uncertainty bounds. This is the standard way to visualize a 2D Gaussian covariance.
3.  **Comparison:** In Q3, the complexity analysis usually shows that the PF is significantly slower than the KF for this specific low-dimensional problem. KF is just a few matrix multiplications ($O(d^3)$ approx), while PF operations scale with the number of particles ($O(N)$).

### **3. How to Run**

1.  Download the three python files.
2.  Run `python q1_sol.py` to see the trajectory simulation and the prediction covariance growth.
3.  Run `python q2_sol.py` to see the effect of measurement updates shrinking the ellipses.
4.  Run `python q3_sol.py` to see the particle cloud evolution and the timing comparison.