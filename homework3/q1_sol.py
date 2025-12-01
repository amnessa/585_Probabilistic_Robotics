import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from pathlib import Path

# Base directory = folder where this script lives
BASE_DIR = Path(__file__).resolve().parent

# Save visuals in a "visuals" subfolder next to this script
outdir = BASE_DIR / "visuals"
outdir.mkdir(exist_ok=True)

# --- CONFIGURATION ---
dt = 1.0
sigma_acc = 1.0  # Standard deviation of acceleration
mu_0 = np.array([0.0, 0.0]) # Initial state [position, velocity]
# Initial covariance (Assume perfect knowledge initially implies 0,
# but usually we use a small epsilon or 0 for theoretical derivation)
Sigma_0 = np.zeros((2, 2))

# --- Q1 (c) System Matrices ---
# State Transition Matrix A
# p_t = p_{t-1} + v_{t-1}*dt
# v_t = v_{t-1}
A = np.array([[1.0, dt],
              [0.0, 1.0]])

# Process Noise Covariance R
# Noise comes from random acceleration 'a'.
# Effect on position: 0.5 * a * dt^2
# Effect on velocity: a * dt
# G is the mapping from noise source to state
G = np.array([[0.5 * dt**2],
              [dt]])
# R = G * sigma_acc^2 * G.T
R = G @ G.T * sigma_acc**2

# --- Helper Function: Uncertainty Ellipse ---
from typing import Any

def plot_covariance_ellipse(mean, cov, ax, n_std: float = 1.0, color: Any = 'b', label: str | None = None):
    """
    Plots an ellipse representing the covariance matrix.
    """
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)

    ell = Ellipse(xy=mean, width=width, height=height, angle=theta,
                  edgecolor=color, facecolor='none', linewidth=2, label=label)
    ax.add_patch(ell)
    return ell

# --- Q1 (b) Simulate True Motion ---
def simulate_trajectory(steps):
    state = mu_0.copy()
    path_x = [state[0]]
    path_v = [state[1]]
    for _ in range(steps):
        acc = np.random.normal(0, sigma_acc)
        state = A @ state + (G * acc).flatten()
        path_x.append(state[0])
        path_v.append(state[1])
    return path_x, path_v

# PLOT 1: Position vs Time (Trajectory)
plt.figure(figsize=(10, 5))
plt.title("Q1(b): True State Trajectories (Position vs Time)")
time_steps = range(6) # t=0 to t=5
for i in range(3):
    px, pv = simulate_trajectory(5)
    plt.plot(time_steps, px, marker='o', linestyle='-', linewidth=2, label=f'Instance {i+1}')

plt.xlabel("Time Step (t)")
plt.ylabel("Position (x)")
plt.xticks(time_steps)
plt.grid(True)
plt.legend()
plt.savefig(outdir / 'q1_plot1.png', dpi=150)
plt.show()

# --- Q1 (d) & (e) KF Prediction Step ---
mu = mu_0.copy()
Sigma = Sigma_0.copy()

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_title("Q1(e): State Prediction Density (1-sigma Ellipses)")
ax.set_xlabel("Position (x)")
ax.set_ylabel("Velocity (x_dot)")
ax.grid(True)

# Plot initial state
ax.plot(mu[0], mu[1], 'ko', label="t=0 Mean")

print("--- Kalman Filter Prediction Step ---")
print(f"t=0: Mean={mu}, Cov=\n{Sigma}")

from typing import List
from numpy.typing import NDArray

means: List[NDArray[np.float64]] = [mu]
covs: List[NDArray[np.float64]] = [Sigma]

for t in range(1, 6):
    # Prediction
    mu_bar = A @ mu
    Sigma_bar = A @ Sigma @ A.T + R

    # Store and Update for next iter
    mu = mu_bar
    Sigma = Sigma_bar
    means.append(mu)
    covs.append(Sigma)

    print(f"t={t}: Mean={mu}, Cov=\n{Sigma}")

    # Plot
    ax.plot(mu[0], mu[1], 'x', label=f"t={t} Mean")
    cmap = plt.get_cmap("viridis")
    plot_covariance_ellipse(mu, Sigma, ax, n_std=1.0, color=cmap(t/5), label=f"t={t} Cov")

# Q1 (f) Correlation Analysis for large t
# Let's simulate further to see correlation
mu_inf = mu
Sigma_inf = Sigma
for _ in range(20):
    Sigma_inf = A @ Sigma_inf @ A.T + R

corr = Sigma_inf[0,1] / (np.sqrt(Sigma_inf[0,0]) * np.sqrt(Sigma_inf[1,1]))
print(f"\nQ1(f): Correlation coefficient at t=25: {corr:.4f}")
print("Interpretation: Position and Velocity become highly positively correlated.")

ax.legend()
plt.axis('equal')
plt.savefig(outdir / 'q1_plot2.png', dpi=150)
plt.show()