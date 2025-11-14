import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# --- Simulation Parameters ---
initial_pose = (0.0, 0.0, np.pi / 4)  # Start at (0,0) facing 45 degrees
v, w, dt = 1.0, 0.5, 0.1             # v=1 m/s, w=0.5 rad/s, dt=0.1 sec
total_time = 5.0                     # Simulate for 5 seconds
robot_radius = 0.25                  # Radius for plotting

def plot_robot_pose(ax, pose, label, color='k', robot_radius=0.15):
    """Helper function to draw a robot as a circle with a heading line."""
    x, y, th = pose
    # Draw the robot's body as a circle
    robot_circle = Circle((x, y), robot_radius, color=color, fill=False, alpha=0.7, lw=1.5)
    ax.add_patch(robot_circle)

    # Draw the heading line (from center to edge)
    ax.plot([x, x + robot_radius * np.cos(th)],
            [y, y + robot_radius * np.sin(th)],
            color=color, lw=2, alpha=0.8)

    # Add a label
    ax.text(x, y - robot_radius * 1.5, label, ha='center', va='top', fontsize=10, color=color)

# Q2(b) — Noisy velocity motion model (canonical 6-alpha version) and sampling
rng = np.random.default_rng(585)

def _sample_var(var: float, rng) -> float:
    return 0.0 if var <= 0.0 else rng.normal(0.0, np.sqrt(var))

def motion_model_noisy(pose, v: float, w: float, dt: float, alphas, rng, eps: float = 1e-9):
    a1,a2,a3,a4,a5,a6 = alphas

    v_var = a1*abs(v) + a2*abs(w)
    w_var = a3*abs(v) + a4*abs(w)
    g_var = a5*abs(v) + a6*abs(w)

    v_hat = v + _sample_var(v_var, rng)
    w_hat = w + _sample_var(w_var, rng)
    gamma_hat = _sample_var(g_var, rng)
    x, y, th = pose
    if abs(w_hat) < eps:
        x_p = x + v_hat * dt * np.cos(th)
        y_p = y + v_hat * dt * np.sin(th)
    else:
        v_over_w = v_hat / w_hat
        th_dt = th + w_hat * dt
        x_p = x - v_over_w * np.sin(th) + v_over_w * np.sin(th_dt)
        y_p = y + v_over_w * np.cos(th) - v_over_w * np.cos(th_dt)

    th_p = (th + w_hat * dt + gamma_hat * dt + np.pi) % (2*np.pi) - np.pi
    return (x_p, y_p, th_p)

def sample_final_poses(initial_pose, v, w, dt, alphas, N: int, rng):
    return np.array([motion_model_noisy(initial_pose, v, w, dt, alphas, rng) for _ in range(N)])

initial_pose = (0.0, 0.0, 0.0)
v, w, dt = 1.0, 0.5, 2
N = 700

alpha_sets = {
    "A: small & balanced": (1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4),
    "B: rotation-dominant": (1e-4, 1e-4, 2e-2, 2e-2, 2e-2, 2e-2),
    "C: translation-dominant": (1e-2, 1e-2, 1e-4, 1e-4, 1e-4, 1e-4),
}

# --- Plotting ---

# Pre-sample to compute global bounds for all plots
all_poses_list = []
for alphas in alpha_sets.values():
    poses_tmp = sample_final_poses(initial_pose, v, w, dt, alphas, N, rng)
    all_poses_list.append(poses_tmp)

all_poses = np.vstack(all_poses_list)
xmin, xmax = all_poses[:,0].min(), all_poses[:,0].max()
ymin, ymax = all_poses[:,1].min(), all_poses[:,1].max()

# Add padding to the limits
pad_x = 0.1 * (xmax - xmin if xmax > xmin else 1.0)
pad_y = 0.2 * (ymax - ymin if ymax > ymin else 1.0)

fig, axs = plt.subplots(1, 3, figsize=(20, 7), sharex=True, sharey=True, dpi=120)
fig.suptitle(f'Q2(b): Noisy Velocity Model (N={N} samples)', fontsize=18, y=1.02)

for ax, (name, alphas) in zip(axs, alpha_sets.items()):
    # Re-sample for this plot
    poses = sample_final_poses(initial_pose, v, w, dt, alphas, N, rng)

    # Plot the cloud of final poses (end poses)
    ax.scatter(poses[:,0], poses[:,1], s=15, alpha=0.4, edgecolors='none', c='blue', label=f'N={N} End Poses')

    # Plot the initial pose (start pose) as a robot
    plot_robot_pose(ax, initial_pose, 'Start Pose', 'g', robot_radius)

    # --- NEW: Calculate and plot the mean end pose ---
    # Calculate mean x and y
    x_mean = np.mean(poses[:, 0])
    y_mean = np.mean(poses[:, 1])

    # Calculate mean theta (correctly handling wrapping)
    th_x_mean = np.mean(np.cos(poses[:, 2]))
    th_y_mean = np.mean(np.sin(poses[:, 2]))
    th_mean = np.arctan2(th_y_mean, th_x_mean)

    mean_end_pose = (x_mean, y_mean, th_mean)
    plot_robot_pose(ax, mean_end_pose, 'Mean End Pose', 'r', robot_radius)
    # --- End New ---

    ax.set_title(name, fontsize=13, pad=10)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_aspect('equal', 'box')
    ax.tick_params(labelsize=11)

    # Set labels
    if ax == axs[0]:
        ax.set_ylabel('Y position (m)', fontsize=12)
    ax.set_xlabel('X position (m)', fontsize=12)
    ax.legend(loc='upper left')

# Apply expanded limits
for ax in axs:
    ax.set_xlim(min(xmin, initial_pose[0]) - pad_x - robot_radius,
                max(xmax, initial_pose[0]) + pad_x + robot_radius)
    ax.set_ylim(min(ymin, initial_pose[1]) - pad_y - robot_radius,
                max(ymax, initial_pose[1]) + pad_y + robot_radius)

plt.tight_layout(pad=1.5)
plt.show()