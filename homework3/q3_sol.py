import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import time
from pathlib import Path

# Base directory = folder where this script lives
BASE_DIR = Path(__file__).resolve().parent

# Save visuals in a "visuals" subfolder next to this script
outdir = BASE_DIR / "visuals"
outdir.mkdir(exist_ok=True)


# --- SETUP ---
dt = 1.0
sigma_acc = 1.0
sigma_meas = np.sqrt(10)
A = np.array([[1.0, dt], [0.0, 1.0]])
# G is implicitly handled in the sampling step

# --- Q3 (a) Functions ---

def sample_motion_model(particles, dt, sigma_acc):
    """
    Propagates particles forward in time.
    x_t = A x_{t-1} + B u_t + noise
    Here, noise enters via acceleration.
    """
    N = particles.shape[0]
    # Random accelerations for each particle
    acc = np.random.normal(0, sigma_acc, N)

    # Update rules:
    # p_new = p + v*dt + 0.5*a*dt^2
    # v_new = v + a*dt

    p_old = particles[:, 0]
    v_old = particles[:, 1]

    p_new = p_old + v_old * dt + 0.5 * acc * dt**2
    v_new = v_old + acc * dt

    new_particles = np.column_stack((p_new, v_new))
    return new_particles

def measurement_model(z, particles, sigma_meas):
    """
    Calculates weights based on likelihood P(z|x).
    Likelihood is Gaussian centered at particle's position.
    """
    # Measurement z corresponds to position (particle[:, 0])
    predicted_pos = particles[:, 0]

    # Calculate Gaussian PDF value
    weights = norm.pdf(z, loc=predicted_pos, scale=sigma_meas)

    # Avoid division by zero (add epsilon)
    weights += 1.e-300
    return weights

def low_variance_sampler(particles, weights):
    """
    Resamples particles based on weights.
    Complexity O(M).
    """
    M = particles.shape[0]
    new_particles = np.zeros_like(particles)

    # Normalize weights
    weights = weights / np.sum(weights)

    r = np.random.uniform(0, 1.0/M)
    c = weights[0]
    i = 0

    for m in range(M):
        U = r + m * (1.0/M)
        while U > c:
            i = (i + 1) % M
            c += weights[i]
        new_particles[m] = particles[i]

    return new_particles

# --- Q3 (c) Particle Filter Prediction Only ---
# Running just prediction for 5 steps, N=1000
N = 1000
particles = np.zeros((N, 2)) # Initialize at [0,0]

plt.figure(figsize=(10, 6))
plt.title("Q3(c): PF Prediction Step (Particles)")
plt.xlabel("Position")
plt.ylabel("Velocity")
plt.plot(particles[:, 0], particles[:, 1], 'k.', alpha=0.1, label='t=0')

for t in range(1, 6):
    particles = sample_motion_model(particles, dt, sigma_acc)
    # Plot
    color = plt.get_cmap('viridis')(t/5)
    plt.plot(particles[:, 0], particles[:, 1], '.', color=color, alpha=0.3, label=f't={t}')

plt.legend()
plt.grid(True)
plt.savefig(outdir / 'q3_plot1.png', dpi=150)
plt.show()

# --- Q3 (d) Particle Filter with Updates ---
# We need the same measurements used in Q2(d) to make a valid comparison,
# but since we are just illustrating, we will generate a single measurement sequence.
# Or better, let's use a simple dummy sequence like z=0, 0, ... or random.
# Let's stick to the problem statement implies "repeat Q2(d)" logic.
# In Q2(d) simulation code, we generated measurements. Let's regenerate here for self-containment.

# Generate Ground Truth
np.random.seed(42)
true_state = np.array([0.0, 0.0])
measurements = []
for _ in range(5):
    acc = np.random.normal(0, sigma_acc)
    # Update true state
    p_new = true_state[0] + true_state[1]*dt + 0.5*acc*dt**2
    v_new = true_state[1] + acc*dt
    true_state = np.array([p_new, v_new])
    # Measure
    z = true_state[0] + np.random.normal(0, sigma_meas)
    measurements.append(z)

# Run PF
particles = np.zeros((N, 2))
plt.figure(figsize=(10, 6))
plt.title("Q3(d): PF with Measurement Updates")
plt.xlabel("Position")
plt.ylabel("Velocity")

for t, z in enumerate(measurements, 1):
    # 1. Prediction
    particles_pred = sample_motion_model(particles, dt, sigma_acc)

    # 2. Weighting
    weights = measurement_model(z, particles_pred, sigma_meas)

    # 3. Resampling
    particles = low_variance_sampler(particles_pred, weights)

    # Plot
    color = plt.get_cmap('jet')(t/5)
    plt.plot(particles[:, 0], particles[:, 1], '.', color=color, alpha=0.2, label=f't={t}')

plt.legend()
plt.grid(True)
plt.savefig(outdir / 'q3_plot2.png', dpi=150)
plt.show()

# --- Q3 (e) Complexity Comparison ---
# We will measure time for 1 iteration of KF vs PF (N=1000)

# KF Timing
t_start = time.time()
runs = 1000 # Run multiple times to average small duration
mu = np.array([0., 0.])
Sigma = np.eye(2)
C = np.array([[1, 0]])
Q = np.array([[10]])
R = np.eye(2) # simplified R
z = 5.0
for _ in range(runs):
    # Pred
    mu_bar = A @ mu
    Sigma_bar = A @ Sigma @ A.T + R
    # Upd
    S = C @ Sigma_bar @ C.T + Q
    K = Sigma_bar @ C.T @ np.linalg.inv(S)
    mu = mu_bar + (K @ (z - C @ mu_bar)).flatten()
    Sigma = (np.eye(2) - K @ C) @ Sigma_bar
t_kf = (time.time() - t_start) / runs

# PF Timing
t_start = time.time()
particles = np.zeros((N, 2))
for _ in range(runs):
    # Pred
    pp = sample_motion_model(particles, dt, sigma_acc)
    # Weight
    w = measurement_model(z, pp, sigma_meas)
    # Resample
    particles = low_variance_sampler(pp, w)
t_pf = (time.time() - t_start) / runs

print(f"--- Q3(e) Complexity Analysis ---")
print(f"Avg Time for 1 KF Iteration: {t_kf:.6f} seconds")
print(f"Avg Time for 1 PF Iteration (N={N}): {t_pf:.6f} seconds")
print(f"Ratio (PF/KF): {t_pf/t_kf:.2f}")