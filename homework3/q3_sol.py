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
N = 1000 # Number of particles

# G is implicitly handled in the sampling step

# --- Q3 (a) Functions ---

def sample_motion_model(particles, dt, sigma_acc):
    """
    Propagates particles forward.
    State: [position, velocity]
    """
    num_p = particles.shape[0]
    # Random acceleration for each particle
    acc = np.random.normal(0, sigma_acc, num_p)

    p_old = particles[:, 0]
    v_old = particles[:, 1]

    # Physics update
    p_new = p_old + v_old * dt + 0.5 * acc * dt**2
    v_new = v_old + acc * dt

    return np.column_stack((p_new, v_new))

def measurement_model(z, particles, sigma_meas):
    """
    Calculates weights P(z|x).
    """
    pred_pos = particles[:, 0]
    # Gaussian likelihood
    weights = norm.pdf(z, loc=pred_pos, scale=sigma_meas)
    # Avoid zero weights
    weights += 1.e-300
    return weights

def low_variance_sampler(particles, weights):
    """
    Resamples particles based on weights.
    """
    M = particles.shape[0]
    new_particles = np.zeros_like(particles)

    # Normalize
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

# --- SIMULATION (Ground Truth) ---
np.random.seed(42) # For reproducibility
true_state = np.array([0.0, 0.0]) # x, x_dot
measurements = []
true_path = [true_state]

for t in range(5):
    acc = np.random.normal(0, sigma_acc)
    # Update state
    p_new = true_state[0] + true_state[1]*dt + 0.5*acc*dt**2
    v_new = true_state[1] + acc*dt
    true_state = np.array([p_new, v_new])
    true_path.append(true_state)
    # Generate noisy measurement
    z = true_state[0] + np.random.normal(0, sigma_meas)
    measurements.append(z)

true_path = np.array(true_path)

# --- Q3 (c) Particle Filter Prediction Only ---

print("Generating Q3(c) Prediction Plots...")
particles = np.zeros((N, 2)) # Init at 0,0

fig_c, axes_c = plt.subplots(1, 5, figsize=(20, 5), sharey=True)
fig_c.suptitle("Q3(c): Particle Filter Prediction (Phase Space Evolution)", fontsize=16)

for t in range(5):
    # Motion Update
    particles = sample_motion_model(particles, dt, sigma_acc)

    # Plotting
    ax = axes_c[t]
    ax.scatter(particles[:, 0], particles[:, 1], s=10, color='orange', alpha=0.5, label='Particles')
    ax.set_title(f"Time t={t+1}")
    ax.set_xlabel("Position (x)")
    if t == 0: ax.set_ylabel("Velocity (x_dot)")
    ax.grid(True)

    # Add statistics for clarity
    mean_pos = np.mean(particles[:,0])
    mean_vel = np.mean(particles[:,1])
    ax.legend(loc='upper left')

plt.tight_layout()
plt.savefig(outdir / 'q3_plot1.png', dpi=150)
plt.show()


# --- Q3 (d)  Particle Filter with Updates ---

print("Generating Q3(d) Update Plots...")
# Reset particles
particles = np.zeros((N, 2))

fig_d, axes_d = plt.subplots(1, 5, figsize=(20, 5), sharey=True)
fig_d.suptitle("Q3(d): Prediction vs Correction (Measurement Update)", fontsize=16)

for t in range(5):
    z = measurements[t]

    # 1. Prediction (Prior)
    particles_bar = sample_motion_model(particles, dt, sigma_acc)

    # Plot Prediction (Low Alpha, Red)
    ax = axes_d[t]
    ax.scatter(particles_bar[:, 0], particles_bar[:, 1], s=15, color='red', alpha=0.15, label='Prediction (Prior)')

    # 2. Weighting
    weights = measurement_model(z, particles_bar, sigma_meas)

    # 3. Resampling (Posterior)
    particles = low_variance_sampler(particles_bar, weights)

    # Plot Correction (High Alpha, Blue)
    ax.scatter(particles[:, 0], particles[:, 1], s=15, color='blue', alpha=0.6, label='Correction (Posterior)')

    # Plot Measurement Line
    ax.axvline(z, color='green', linestyle='--', linewidth=2, label=f'Meas z={z:.1f}')

    # Plot True State
    ax.plot(true_path[t+1, 0], true_path[t+1, 1], 'k*', markersize=12, label='Truth')

    ax.set_title(f"t={t+1}")
    ax.set_xlabel("Position (x)")
    if t == 0: ax.set_ylabel("Velocity (x_dot)")
    ax.grid(True)

    # Only add legend to the first plot to reduce clutter
    if t == 0:
        ax.legend(loc='upper left', fontsize='small')

plt.tight_layout()
plt.savefig(outdir / 'q3_plot2.png', dpi=150)
plt.show()

# --- Q3 (e) Complexity Comparison ---
# Re-measuring to match your requested output format
print("\n--- Q3(e) Complexity Analysis (Recalculating) ---")

# KF Timing
t_start = time.time()
runs = 1000
# KF vars
mu_kf = np.array([0., 0.])
Sigma_kf = np.eye(2)
Q_kf = np.array([[10]])
C_kf = np.array([[1, 0]])
G_kf = np.array([[0.5], [1]])
R_kf = G_kf @ G_kf.T # simplified
z_val = 5.0

for _ in range(runs):
    # Predict
    mu_bar = A @ mu_kf
    Sigma_bar = A @ Sigma_kf @ A.T + R_kf
    # Update
    S = C_kf @ Sigma_bar @ C_kf.T + Q_kf
    K = Sigma_bar @ C_kf.T @ np.linalg.inv(S)
    mu_kf = mu_bar + (K @ (z_val - C_kf @ mu_bar)).flatten()
    Sigma_kf = (np.eye(2) - K @ C_kf) @ Sigma_bar
t_kf = (time.time() - t_start) / runs

# PF Timing
t_start = time.time()
# Reset PF
parts_timed = np.zeros((N, 2))
for _ in range(runs):
    # Predict
    pp = sample_motion_model(parts_timed, dt, sigma_acc)
    # Weight
    w = measurement_model(z_val, pp, sigma_meas)
    # Resample
    parts_timed = low_variance_sampler(pp, w)
t_pf = (time.time() - t_start) / runs

print(f"Avg Time for 1 KF Iteration: {t_kf:.6f} seconds")
print(f"Avg Time for 1 PF Iteration (N={N}): {t_pf:.6f} seconds")
print(f"Ratio (PF/KF): {t_pf/t_kf:.2f}")