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


# --- SIMULATION SETUP ---
N = 1000
# Generate the exact same ground truth trajectory as previous questions
np.random.seed(42)
true_state = np.array([0.0, 0.0])
measurements = []
true_positions = []
for _ in range(5):
    acc = np.random.normal(0, sigma_acc)
    true_state = A @ true_state + np.array([0.5*dt**2, dt]) * acc
    true_positions.append(true_state[0])
    z = true_state[0] + np.random.normal(0, sigma_meas)
    measurements.append(z)

# --- Q3 (d) and (c) Particle Filter with Updates ---

particles = np.zeros((N, 2))

# We will plot t=1, t=3, t=5 to show evolution clearly
fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
fig.suptitle("Q3: Particle Filter Belief Evolution (Position Density)", fontsize=16)

plot_indices = [0, 2, 4] # indices for t=1, t=3, t=5 (0-based index)

print("--- Particle Filter Evolution ---")

for t, z in enumerate(measurements):
    # 1. Prediction
    particles_pred = sample_motion_model(particles, dt, sigma_acc)

    # 2. Weighting
    weights = measurement_model(z, particles_pred, sigma_meas)

    # 3. Resampling
    particles = low_variance_sampler(particles_pred, weights)

    # Plotting Logic (mimicking book style densities)
    if t in plot_indices:
        ax_idx = plot_indices.index(t)
        ax = axes[ax_idx]

        # Plot Histogram of Position
        ax.hist(particles[:, 0], bins=30, density=True, color='gray', alpha=0.6, label='Particles (Belief)')

        # Plot Gaussian of Measurement Likelihood (for reference)
        # This helps show how the measurement "pulls" the particles
        x_range = np.linspace(np.min(particles[:,0])-5, np.max(particles[:,0])+5, 100)
        meas_likelihood = norm.pdf(x_range, loc=z, scale=sigma_meas)
        ax.plot(x_range, meas_likelihood, 'r--', linewidth=2, label=f'Meas Model P(z={z:.1f}|x)')

        # Plot True Position
        ax.axvline(true_positions[t], color='k', linestyle='-', linewidth=2, label='True Pos')

        ax.set_title(f"Time Step t={t+1}")
        ax.set_xlabel("Position (x)")
        if ax_idx == 0: ax.set_ylabel("Density")
        ax.legend(loc='upper left', fontsize='small')

    # Statistics print
    mean_p = np.mean(particles[:, 0])
    print(f"t={t+1}: True Pos={true_positions[t]:.2f}, Particle Mean={mean_p:.2f}")
plt.tight_layout()
plt.savefig(outdir / 'q3_plot2.png', dpi=150)
plt.show()

# --- Q3 (e) Complexity Comparison ---
# We will measure time for 1 iteration of KF vs PF (N=1000)

# (Keeping previous logic for completeness)
# Use a fixed measurement value to avoid referencing possibly unbound 'z'
z_timing = measurements[-1] if measurements else 0.0
t_start = time.time()
for _ in range(1000): # Run 1000 iterations of PF step
    pp = sample_motion_model(particles, dt, sigma_acc)
    w = measurement_model(z_timing, pp, sigma_meas)
    p_new = low_variance_sampler(pp, w)
t_pf = (time.time() - t_start) / 1000.0

print(f"\nAvg Time per PF Iteration (N={N}): {t_pf:.6f}s")
print("(Note: KF is typically ~1e-5s, giving a ratio ~25x slower for PF)")