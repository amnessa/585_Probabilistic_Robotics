import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# --- IMPORT SETUP FROM Q1 ---
dt = 1.0
sigma_acc = 1.0
A = np.array([[1.0, dt], [0.0, 1.0]])
G = np.array([[0.5 * dt**2], [dt]])
R = G @ G.T * sigma_acc**2
mu_0 = np.array([0.0, 0.0])
Sigma_0 = np.zeros((2, 2))

# --- HELPER PLOT FUNCTION ---
def plot_ellipse(mean, cov, ax, color, style='-', label=None):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * 1.0 * np.sqrt(vals) # 1-sigma
    ell = Ellipse(xy=mean, width=width, height=height, angle=theta,
                  edgecolor=color, linestyle=style, facecolor='none', linewidth=2, label=label)
    ax.add_patch(ell)

# --- Q2 (a) Measurement Model ---
# We measure position x. z_t = C x_t + delta.
# x_t = [p, v]. So C = [1, 0]
C = np.array([[1.0, 0.0]])
sigma_meas = np.sqrt(10) # Variance is 10
Q = np.array([[10.0]])

# --- Q2 (b) Update at t=5 with z=5 ---
# First, we need the prediction at t=5 (from Q1 logic)
mu_pred = mu_0
Sigma_pred = Sigma_0

# Predict up to t=5
for _ in range(5):
    mu_pred = A @ mu_pred
    Sigma_pred = A @ Sigma_pred @ A.T + R

print("--- Q2(b) Single Update at t=5 ---")
print(f"Prediction at t=5: Mean={mu_pred}")
print(f"Pred Covariance:\n{Sigma_pred}")

# Measurement
z = 5.0

# Kalman Gain
# K = Sigma_bar * C.T * inv(C * Sigma_bar * C.T + Q)
S = C @ Sigma_pred @ C.T + Q # Innovation covariance
K = Sigma_pred @ C.T @ np.linalg.inv(S)

print(f"Kalman Gain K:\n{K}")

# Update
mu_updated = mu_pred + (K @ (z - C @ mu_pred)).flatten()
Sigma_updated = (np.eye(2) - K @ C) @ Sigma_pred

print(f"Updated Mean: {mu_updated}")
print(f"Updated Covariance:\n{Sigma_updated}")

# Plotting Q2(b)
fig, ax = plt.subplots(figsize=(8, 6))
ax.grid(True)
plot_ellipse(mu_pred, Sigma_pred, ax, 'red', '--', "Prediction (Prior)")
ax.plot(mu_pred[0], mu_pred[1], 'rx')
plot_ellipse(mu_updated, Sigma_updated, ax, 'blue', '-', "Updated (Posterior)")
ax.plot(mu_updated[0], mu_updated[1], 'bx')
# Plot measurement line
ax.axvline(z, color='green', label=f'Measurement z={z}')
ax.set_title("Q2(b): Update at t=5")
ax.set_xlabel("Position")
ax.set_ylabel("Velocity")
ax.legend()
plt.show()


# --- Q2 (d) Filter with measurements at every step ---
# Let's assume we measure '0' at every step just to see the ellipse evolution,
# or we can simulate a trajectory. Let's simulate a trajectory to get valid Zs.
np.random.seed(42)
true_states = [mu_0]
measurements = []
# Generate ground truth and measurements
curr_state = mu_0
for _ in range(5):
    acc = np.random.normal(0, sigma_acc)
    curr_state = A @ curr_state + (G * acc).flatten()
    true_states.append(curr_state)
    # Measurement z = C*x + noise
    z_val = (C @ curr_state)[0] + np.random.normal(0, sigma_meas)
    measurements.append(z_val)

mu = mu_0
Sigma = Sigma_0

fig2, ax2 = plt.subplots(figsize=(10, 8))
ax2.grid(True)
ax2.set_title("Q2(d): KF Evolution with updates t=1..5")
ax2.set_xlabel("Position")
ax2.set_ylabel("Velocity")

plot_ellipse(mu, Sigma, ax2, 'k', label="t=0")

for t, z_val in enumerate(measurements, 1):
    # 1. Prediction
    mu_bar = A @ mu
    Sigma_bar = A @ Sigma @ A.T + R

    # 2. Update
    S = C @ Sigma_bar @ C.T + Q
    K = Sigma_bar @ C.T @ np.linalg.inv(S)
    mu = mu_bar + (K @ (z_val - C @ mu_bar)).flatten()
    Sigma = (np.eye(2) - K @ C) @ Sigma_bar

    # Plot
    cmap = plt.cm.get_cmap('jet')
    color = cmap(t/5)
    plot_ellipse(mu, Sigma, ax2, color, label=f"t={t} (Posterior)")
    ax2.plot(mu[0], mu[1], 'o', color=color)

ax2.legend()
plt.show()