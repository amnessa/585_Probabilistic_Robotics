# **EE 585: Homework 2 \- Investigation and Implementation Plan**

This document first investigates each homework question using your provided lecture slides and the complete textbook PDF, then provides a structured plan for implementation.

## **Part 1: Homework Investigation**

Here is a breakdown of each question from ee585\_Hw2\_v3.pdf and where to find the answers in your course materials.

* **Due Date:** Nov 19, 2025 (Wed) 23:59
* **Submission:** A single .zip file (Surname\_hw2.zip) containing your PDF report (Surname\_Hw2.pdf) and all Python files.

### **Q1: Markov Weather Simulator**

* **Source:** The homework PDF states this is "Chapter 1, Part of Ex. 2 from your textbook."
* **Investigation:** This appears to be a typo in the homework PDF. The correct reference is **Chapter 2, Exercise 2** on **page 36** of your new textbook (Probabilistic Robotics (2005, The MIT Press) Full\_book.pdf). This exercise matches the sunny/cloudy/rainy problem exactly.
* **Task:** Part (a) is a manual probability calculation. Part (b) requires you to write a Python simulator using the provided transition table.

### **Q2: Velocity Motion Model**

* **Source:** The homework explicitly references "Page 34" and "Page 39" of your lecture slides.
* **Investigation:** These references are found in ProRob\_RobotMotion\_Fall25\_v9.pdf.
  * **Page 34:** "Velocity Motion Model \- Geometry" provides the equations for the "ideal" model for Q2(a).
  * **Page 36:** "Algorithm sample\_motion\_model\_velocity" provides the *exact* algorithm for the "noisy" model for Q2(b). This is also in your textbook on **Page 124, Table 5.3**.
  * **Page 39:** "Examples: Velocity Based" shows the three target plots you need to replicate.
* **Task:** Implement the two motion models (ideal and noisy sampling) in Python. You will use these to generate plots for your report.

### **Q3: 1D Robot Dynamics**

* **Source:** The homework states "Solve Chapter 5, Question 1 From your Texbook."
* **Investigation:** Using the new textbook PDF, this corresponds to **Chapter 5, Exercise 1** on **page 145**.
* **Task:** This is a purely theoretical problem. The question asks you to add dynamics (velocity and acceleration) to a 1D robot's state and derive the new motion model. This will be a mathematical derivation for your PDF report.

### **Q4: Continuous Camera Sensor Model**

* **Source:** The homework asks you to research the "pinhole camera model" and derive a sensor model p(z|x,m).
* **Investigation:** This is the most complex question.
  * The geometric "pinhole model" is a research task.
  * The *derivation* is an analogy to the landmark models discussed in your lectures and textbook. The most relevant algorithm is landmark\_model\_known\_correspondence from ProRob\_SensorModels\_Fall25\_v12.pdf (Page 46\) and your textbook (**Page 178, Table 6.4**).
  * The *sampling* part (Q4e) is analogous to the algorithm sample\_landmark\_model\_known\_correspondence from your textbook (**Page 180, Table 6.5**).
  * Your lecturer's notes on the "uncertainty cone" are the key to answering Q4(e) and Q4(f) correctly.

## **Part 2: Structured Implementation Plan**

### **Q1: Markov Weather Simulator**

**Relevant Materials:**

* **Textbook:** Probabilistic Robotics...Full\_book.pdf, **Chapter 2, Exercise 2(a-c), page 36**.
* **Lecture Slides:** ProRob\_BayesFilters\_Fall25\_v11.pdf (Pages 50-51, Markov Assumption).

#### **(a) Probability Calculation**

* **Objective:** Calculate P(Day2=C, Day3=C, Day4=R | Day1=S).
* **Task:** Manual calculation for your PDF report.
* **Derivation:**
  1. This is a Markov chain, so the probability of a sequence is the product of the individual transitions.
  2. P(Day2=C | Day1=S) \= 0.2 (from table in homework PDF)
  3. P(Day3=C | Day2=C) \= 0.4
  4. P(Day4=R | Day3=C) \= 0.2
  5. P(sequence) \= 0.2 \* 0.4 \* 0.2 \= 0.016
* **Report Section:** Write this derivation in your PDF.

#### **(b) Python Simulator**

* **Objective:** Write a Python script to generate random weather sequences.
* **Implementation:**
  1. Import numpy.
  2. Define states: states \= \['sunny', 'cloudy', 'rainy'\].
  3. Define the 3x3 transition probability matrix P from the homework.
  4. Initialize an empty list sequence and set an initial state (e.g., current\_state\_index \= 0 for 'sunny').
  5. Write a for loop to iterate N times (e.g., N=1000).
  6. Inside the loop, get the probability row for the current state: probabilities \= P\[current\_state\_index\].
  7. Use numpy.random.choice() to select the next state:
     next\_state\_index \= np.random.choice(\[0, 1, 2\], p=probabilities)

  8. Append states\[next\_state\_index\] to your sequence.
  9. Update current\_state\_index \= next\_state\_index.
* **Report Section:** Include a histogram (e.g., from matplotlib.pyplot.hist) of the weather distribution from a long simulation (N=10,000). Comment on the resulting stationary distribution (as asked in the textbook version of this exercise).

### **Q2: Velocity Motion Model**

**Relevant Materials:**

* **Lecture Slides (Primary):** ProRob\_RobotMotion\_Fall25\_v9.pdf
* **Textbook (Secondary):** Probabilistic Robotics...Full\_book.pdf (Chapter 5, Section 5.3)

#### **(a) Ideal (Noiseless) Model**

* **Objective:** Implement motion\_model\_ideal(current\_pose, v, w, dt).
* **Source:** ProRob\_RobotMotion\_Fall25\_v9.pdf, **Page 34, "Velocity Motion Model \- Geometry"**.
* **Implementation:**
  1. Your function will take (x, y, theta), v, w, and dt.
  2. **Crucial Step:** Check if w is close to zero (e.g., abs(w) \< 1e-6).
  3. **If w is near zero (straight motion):**
     import numpy as np
     x\_prime \= x \+ v \* dt \* np.cos(theta)
     y\_prime \= y \+ v \* dt \* np.sin(theta)
     theta\_prime \= theta

  4. **If w is not zero (curved motion):** Use the equations from Page 34:
     import numpy as np
     v\_over\_w \= v / w
     x\_prime \= x \- v\_over\_w \* np.sin(theta) \+ v\_over\_w \* np.sin(theta \+ w \* dt)
     y\_prime \= y \+ v\_over\_w \* np.cos(theta) \- v\_over\_w \* np.cos(theta \+ w \* dt)
     theta\_prime \= theta \+ w \* dt

  5. Return (x\_prime, y\_prime, theta\_prime).
* **Simulation & Report:**
  * Start at pose \= \[0, 0, 0\].
  * Create a list path\_history.
  * Loop 50 times, calling pose \= motion\_model\_ideal(pose, v=1.0, w=0.5, dt=0.1) and appending the new pose to path\_history.
  * Plot the (x, y) coordinates from path\_history using matplotlib.pyplot.plot().
  * Use matplotlib.pyplot.quiver() to draw the initial and final poses.

#### **(b) Noisy Model**

* **Objective:** Implement motion\_model\_noisy() by *sampling*.
* **Source:** ProRob\_RobotMotion\_Fall25\_v9.pdf, **Page 36, "Algorithm sample\_motion\_model\_velocity"**.
* **Textbook Source:** Probabilistic Robotics...Full\_book.pdf, **Page 124, Table 5.3**.
* **Implementation:**
  1. Your function will take (x, y, theta), (v, w), dt, and the 6 noise parameters (a1...a6).
  2. Define a helper function sample(b\_squared) that returns a sample from a normal distribution with variance b\_squared: return np.random.normal(0, np.sqrt(b\_squared)).
  3. Implement the algorithm from the slides *exactly*. **Note:** The slides (and textbook) use $\\alpha\_1|v| \+ \\alpha\_2|\\omega|$ as the *variance* parameter. Your homework PDF seems to imply these are *variances*. The textbook Table 5.3 (pg 124\) shows the $sample()$ function applied to $a\_1v^2 \+ a\_2w^2$. Let's follow the **textbook Table 5.3 / slide 36** as it's an explicit algorithm:
     import numpy as np
     \# Helper function to sample from a normal dist with variance b\_squared
     def sample(b\_squared):
         if b\_squared \< 0:
             b\_squared \= 0
         return np.random.normal(0, np.sqrt(b\_squared))

     \# 1\. Calculate noisy velocities
     v\_hat \= v \+ sample(a1\*v\*\*2 \+ a2\*w\*\*2)
     w\_hat \= w \+ sample(a3\*v\*\*2 \+ a4\*w\*\*2)

     \# 2\. Calculate noisy drift
     gamma\_hat \= sample(a5\*v\*\*2 \+ a6\*w\*\*2)

     \# 3\. Get current pose
     x, y, theta \= current\_pose

     \# 4\. Handle singularity (straight line motion)
     if abs(w\_hat) \< 1e-6:
         x\_prime \= x \+ v\_hat \* dt \* np.cos(theta)
         y\_prime \= y \+ v\_hat \* dt \* np.sin(theta)
     else:
         \# 5\. Handle curved motion
         v\_over\_w \= v\_hat / w\_hat
         x\_prime \= x \- v\_over\_w \* np.sin(theta) \+ v\_over\_w \* np.sin(theta \+ w\_hat \* dt)
         y\_prime \= y \+ v\_over\_w \* np.cos(theta) \- v\_over\_w \* np.cos(theta \+ w\_hat \* dt)

     \# 6\. Add final rotation
     theta\_prime \= theta \+ w\_hat \* dt \+ gamma\_hat \* dt

     return (x\_prime, y\_prime, theta\_prime)

* **Simulation & Report:**
  * To replicate the plots on **Page 39** of the motion slides:
  * Set initial\_pose \= (0, 0, 0).
  * Create an empty list final\_poses.
  * Loop N=500 times. In *each* loop, call final\_poses.append(motion\_model\_noisy(initial\_pose, ...))
  * Plot the results using matplotlib.pyplot.scatter() on the (x, y) components of final\_poses.
  * Draw the initial\_pose using quiver().

#### **(c) Analysis**

* **Objective:** Report parameters and discuss.
* **Task:** Generate three plots for your report using different noise parameters to match the figures on **Page 39** of ProRob\_RobotMotion\_Fall25\_v9.pdf.
  1. **Plot 1 (Fig. a):** Small, balanced noise (e.g., a1=0.1, a2=0.1, etc.).
  2. **Plot 2 (Fig. b):** Large orientation noise (e.g., a5=1.0, a6=1.0, others small).
  3. **Plot 3 (Fig. c):** Large velocity noise (e.g., a1=1.0, a2=1.0, a3=1.0, a4=1.0, others small).
* **Report Section:** State your parameters for each plot. Discuss how the shape of the sample cloud (e.g., "banana-shaped," "fanned-out") is caused by the specific noise parameters you used.

### **Q3: 1D Robot Dynamics**

* **Objective:** Solve **Chapter 5, Exercise 1** from your textbook.
* **Source:** Probabilistic Robotics...Full\_book.pdf, **Page 145**.
* **Task:** This is a theoretical derivation for your PDF report. The question is:"This exercise requires you to add dynamics to the 1-D motion model...
  (a) Let the state be augmented by velocity... $x\_t \= (x\_t \\quad \\dot{x}\_t)^T$. Formulate a linear Gaussian motion model... The control $u\_t$ is the acceleration $\\ddot{x}\_t$.
  (b) What is the posterior distribution $p(x\_t | u\_t, x\_{t-1})$?"
* **Report Section:**
  1. **State:** Your state vector is $x\_t \= \\begin{pmatrix} x\_t \\\\ \\dot{x}\_t \\end{pmatrix}$.
  2. **Control:** Your control is $u\_t \= \\ddot{x}\_t$.
  3. **Kinematics:** Use basic physics equations for constant acceleration over an interval $\\Delta t$:
     * $x\_t \= x\_{t-1} \+ \\dot{x}\_{t-1}\\Delta t \+ \\frac{1}{2}\\ddot{x}\_t(\\Delta t)^2$
     * $\\dot{x}\_t \= \\dot{x}\_{t-1} \+ \\ddot{x}\_t\\Delta t$
  4. **Linear Model:** Write this in the linear form $x\_t \= A\_t x\_{t-1} \+ B\_t u\_t \+ \\epsilon\_t$ (from textbook, Eq. 3.2, page 41).
     * $A\_t \= \\begin{pmatrix} 1 & \\Delta t \\\\ 0 & 1 \\end{pmatrix}$
     * $B\_t \= \\begin{pmatrix} \\frac{1}{2}(\\Delta t)^2 \\\\ \\Delta t \\end{pmatrix}$
  5. **Answer:** The posterior $p(x\_t | u\_t, x\_{t-1})$ is a linear Gaussian distribution. It is given by $\\mathcal{N}(x\_t; \\mu\_t, R\_t)$, where the mean is $\\mu\_t \= A\_t x\_{t-1} \+ B\_t u\_t$ and $R\_t$ is the covariance matrix of the process noise $\\epsilon\_t$ (which you can just state as $R\_t$).

### **Q4: Continuous Camera Sensor Model**

* **Objective:** Derive a 2D pinhole camera sensor model.
* **Relevant Materials:**
  * ProRob\_SensorModels\_Fall25\_v12.pdf (especially Page 46).
  * Probabilistic Robotics...Full\_book.pdf (Chapter 6, Section 6.6, Page 176).
  * Your lecturer's notes from the screenshot and transcript.

#### **(a) Pinhole Camera Model**

* **Report Section:** Research and draw the 2D planar pinhole model (like your lecturer's drawing).
  * Label the optical center $O$ (this is the robot's pose $x \= (x\_r, y\_r, \\theta\_r)$).
  * Label the image plane (a 1D line) at focal length $f$.
  * Label a landmark $L$ from the map $m$ at world coordinates $(m\_x, m\_y)$.
  * Derive the deterministic projection $z\_{ideal} \= h(x, m)$. This involves:
    1. Converting the landmark's world coordinates $(m\_x, m\_y)$ to the robot's local coordinates $(L\_{local,x}, L\_{local,y})$ using a rotation and translation based on $x$.
    2. Using similar triangles to project this local 2D point onto the 1D image plane: $z\_{ideal} \= f \\cdot \\frac{L\_{local,y}}{L\_{local,x}}$ (or a similar expression, depending on your coordinate setup).

#### **(b) Noise Model**

* **Report Section:** State the model given in the homework.
  * The measured pixel is $z \= z\_{ideal} \+ \\mathcal{N}(0, \\sigma^2)$.
  * Therefore, the *forward sensor model* is $p(z | x, m) \= \\mathcal{N}(z; h(x, m), \\sigma^2)$.

#### **(c) Procedure**

* **Report Section:** Outline your plan.
  1. **For the forward model (Q4d):** I will derive the deterministic geometric function $z\_{ideal} \= h(x, m)$ from (a) and plug it into the Gaussian PDF from (b) to get the final algorithm p(z | x, m).
  2. **For the sampling model (Q4e):** I will analyze the *inverse* problem, $p(x | z, m)$. I will show that a single pixel measurement $z$ (a 1D value) cannot be used to find the full 3D robot pose $x \= (x\_r, y\_r, \\theta\_r)$. I will use the lecturer's "uncertainty cone" concept to explain what *can* be sampled.

#### **(d) Derivation (Forward Model)**

* **Objective:** Derive algorithm\_camera\_model(z, x, m).
* **Task:** This function just computes the value $p(z | x, m)$.
* **Report Section:**
  1. Your function will be analogous to landmark\_model\_known\_correspondence from your slides (Page 46\) or textbook (**Page 178, Table 6.4**).
  2. **Input:** Robot pose $x$, map $m$ (which gives landmark $L\_j$), and pixel measurement $z$.
  3. **Step 1:** Calculate the *expected* (ideal) pixel measurement, $z\_{ideal} \= h(x, m)$, using your pinhole geometry from (a).
  4. **Step 2:** Calculate the *error* (innovation): $\\Delta z \= z \- z\_{ideal}$.
  5. Step 3: Calculate the probability of this error using the Gaussian PDF:
     prob \= (1 / (sigma \* sqrt(2\*pi))) \* exp(-0.5 \* (delta\_z / sigma)\*\*2)
  6. This prob is the return value of your algorithm.

#### **(e) Derivation (Sampling Model)**

* **Objective:** Derive sample\_robotpose\_camera\_model(...).
* **Source:** Analogy to sample\_landmark\_model\_known\_correspondence from Probabilistic Robotics...Full\_book.pdf (**Page 180, Table 6.5**).
* **Report Section:**
  1. The algorithm in Table 6.5 samples a pose from a *range and bearing* measurement.
  2. A 1D pixel measurement $z$ only provides **bearing** (direction), not range.
  3. **Answer 1:** "Would this function be able to sample absolute robot poses?" **No.** A single pixel measurement from a single landmark only constrains the pose to a *line* in the world (the "bearing line"). The robot could be anywhere on this line.
  4. **Answer 2:** "What can you sample?" You can sample the robot's **orientation** $\\theta$ (bearing). The Gaussian noise $\\mathcal{N}(z; \\dots)$ on the 1D pixel plane maps to a (non-Gaussian) angular distribution for $\\theta$ in the world.
  5. Your sample\_robotpose\_camera\_model(z, m) function would:
     1. Sample a noisy pixel: z\_sample \= z \+ np.random.normal(0, sigma).
     2. Back-project this z\_sample from the image plane, through the optical center $O$, to get a bearing line (an angle) in world coordinates.
     3. This sampled angle is a sample of the robot's possible orientation, but its (x, y) position remains unconstrained along that line.

#### **(f) Discussion**

* **Objective:** Discuss difficulties.
* **Task:** Use your lecturer's explanation from the transcript and drawing.
* **Report Section:**
  * The main difficulty is the **asymmetry and non-linearity of the problem** (as explained by your lecturer).
  * **Forward Model p(z|x, m) (Q4d):** This is easy. The physics are causal. You start in the 2D world, project to 1D pixels, and add 1D Gaussian noise.
  * **Inverse Model p(x|z, m) (Q4e):** This is hard. You have to invert the model. You start with a 1D Gaussian distribution on the pixel plane and back-project it into the 2D world.
  * As your lecturer explained \[01:27 \- 02:05\], this projection is **non-linear**. A simple Gaussian in pixel-space becomes a "weird density" (garip bir density) \[01:45\] or an "uncertainty cone" (belirsizlik bölgesi) \[01:40\] in the 2D world-space.
  * Therefore, the central difficulty is: **The belief bel(x) \= p(x|z,m) is not Gaussian**, even though the sensor noise itself is. This makes it difficult to represent and sample from, which is a key issue in probabilistic robotics.

