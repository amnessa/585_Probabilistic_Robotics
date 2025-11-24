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

# Visualization: Use subplots to avoid overlap
fig, axes = plt.subplots(2, 3, figsize=(15, 8))
fig.suptitle("Q3(c): PF Prediction Step Evolution (Phase Space)", fontsize=16)
axes = axes.flatten()

# Plot t=0
axes[0].plot(particles[:, 0], particles[:, 1], 'k.', alpha=0.2)
axes[0].set_title("t=0 (Initial)")
axes[0].set_xlabel("Position")
axes[0].set_ylabel("Velocity")
axes[0].grid(True)

for t in range(1, 6):
    particles = sample_motion_model(particles, dt, sigma_acc)

    # Plot in subplot
    ax = axes[t]
    cmap = plt.get_cmap("viridis")
    color = cmap(t/5)
    ax.plot(particles[:, 0], particles[:, 1], '.', color=color, alpha=0.4, markersize=3)
    ax.set_title(f"t={t}")
    ax.set_xlabel("Position")
    ax.set_ylabel("Velocity")
    ax.grid(True)

    # Calculate and show statistics
    mean_pos = np.mean(particles[:, 0])
    var_pos = np.var(particles[:, 0])
    ax.text(0.05, 0.9, f"Mean P: {mean_pos:.2f}\nVar P: {var_pos:.2f}",
            transform=ax.transAxes, bbox=dict(facecolor='white', alpha=0.8))

plt.tight_layout()
plt.savefig(outdir / 'q3_plot1.png', dpi=150)
plt.show()

# --- Q3 (d) Particle Filter with Updates ---

# Generate Ground Truth
np.random.seed(42)
true_state = np.array([0.0, 0.0])
true_path = [true_state]
measurements = []
for _ in range(5):
    acc = np.random.normal(0, sigma_acc)
    # Update true state
    p_new = true_state[0] + true_state[1]*dt + 0.5*acc*dt**2
    v_new = true_state[1] + acc*dt
    true_state = np.array([p_new, v_new])
    true_path.append(true_state)
    # Measure
    z = true_state[0] + np.random.normal(0, sigma_meas)
    measurements.append(z)
true_path = np.array(true_path)

# Run PF
particles = np.zeros((N, 2))
history = [(0, particles.copy())] # Store history for Time-vs-Position plot

# Figure for Phase Space Evolution
fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
fig2.suptitle("Q3(d): PF Posterior Evolution (Phase Space)", fontsize=16)
axes2 = axes2.flatten()

# Plot t=0
axes2[0].plot(particles[:, 0], particles[:, 1], 'k.', alpha=0.2)
axes2[0].plot(true_path[0,0], true_path[0,1], 'r*', markersize=10, label='True')
axes2[0].set_title("t=0 (Prior)")
axes2[0].grid(True)

print("\n--- Q3(d) Particle Statistics ---")

for t, z in enumerate(measurements, 1):
    # 1. Prediction
    particles_pred = sample_motion_model(particles, dt, sigma_acc)

    # 2. Weighting
    weights = measurement_model(z, particles_pred, sigma_meas)

    # 3. Resampling
    particles = low_variance_sampler(particles_pred, weights)
    history.append((t, particles.copy()))

    # Statistics
    mean_p = np.mean(particles[:, 0])
    std_p = np.std(particles[:, 0])
    print(f"t={t}: True Pos={true_path[t,0]:.2f}, Est Mean={mean_p:.2f}, Est Std={std_p:.2f}")

    # Plotting Phase Space (Subplot)
    ax = axes2[t]
    cmap = plt.get_cmap("viridis")
    color = cmap(t/5)

    # Plot particles
    ax.plot(particles[:, 0], particles[:, 1], '.', color=color, alpha=0.3, markersize=3, label='Particles')
    # Plot Ground Truth
    ax.plot(true_path[t, 0], true_path[t, 1], 'r*', markersize=12, markeredgecolor='k', label='Truth')
    # Plot Measurement
    ax.axvline(z, color='green', linestyle='--', alpha=0.7, label=f'Meas z={z:.2f}')

    ax.set_title(f"t={t}")
    ax.set_xlabel("Position")
    ax.set_ylabel("Velocity")
    ax.grid(True)
    if t == 1: ax.legend()

plt.tight_layout()
plt.savefig(outdir / 'q3_plot2.png', dpi=150)
plt.show()

# --- ALTERNATIVE VISUALIZATION (Textbook Style: Time vs Position) ---
# This mimics Figure 5.10 in Probabilistic Robotics (Time on X-axis, Position on Y-axis)
plt.figure(figsize=(12, 6))
plt.title("Q3(d) Alternative: Trajectory Tracking (Time vs Position)")

# Plot particles for each time step
for t, parts in history:
    # Add jitter to t for visualization density
    t_jitter = t + np.random.normal(0, 0.04, size=parts.shape[0])
    plt.plot(t_jitter, parts[:, 0], '.', color='gray', alpha=0.1, zorder=1)

# Plot True Path
plt.plot(range(6), true_path[:, 0], 'r-o', linewidth=2, label='True Path', zorder=2)

# Plot Measurements
plt.plot(range(1, 6), measurements, 'gx', markersize=10, label='Measurements', zorder=3)

plt.xlabel("Time Step (t)")
plt.ylabel("Position (x)")
plt.legend()
plt.grid(True)
plt.xticks(range(6))
plt.savefig(outdir / 'q3_plot3.png', dpi=150)
plt.show()

# --- Q3 (e) Complexity Comparison ---
# We will measure time for 1 iteration of KF vs PF (N=1000)

# KF Timing
t_start = time.time()
runs = 1000
mu = np.array([0., 0.])
Sigma = np.eye(2)
C = np.array([[1, 0]])
Q = np.array([[10]])
R = np.eye(2)
z_val = 5.0
for _ in range(runs):
    # Pred
    mu_bar = A @ mu
    Sigma_bar = A @ Sigma @ A.T + R
    # Upd
    S = C @ Sigma_bar @ C.T + Q
    K = Sigma_bar @ C.T @ np.linalg.inv(S)
    mu = mu_bar + (K @ (z_val - C @ mu_bar)).flatten()
    Sigma = (np.eye(2) - K @ C) @ Sigma_bar
t_kf = (time.time() - t_start) / runs

# PF Timing
t_start = time.time()
# Re-init particles for fairness
particles_timing = np.zeros((N, 2))
for _ in range(runs):
    # Pred
    pp = sample_motion_model(particles_timing, dt, sigma_acc)
    # Weight
    w = measurement_model(z_val, pp, sigma_meas)
    # Resample
    particles_timing = low_variance_sampler(pp, w)
t_pf = (time.time() - t_start) / runs

print(f"\n--- Q3(e) Complexity Analysis ---")
print(f"Avg Time for 1 KF Iteration: {t_kf:.6f} seconds")
print(f"Avg Time for 1 PF Iteration (N={N}): {t_pf:.6f} seconds")
print(f"Ratio (PF/KF): {t_pf/t_kf:.2f}")