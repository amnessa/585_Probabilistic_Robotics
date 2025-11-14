# Q2(a) — Ideal velocity motion model and a simple trajectory demo
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def wrap_angle(theta: float) -> float:
    return (theta + np.pi) % (2*np.pi) - np.pi

def motion_model_ideal(pose, v: float, w: float, dt: float, eps: float = 1e-9):
    x, y, th = pose
    if abs(w) < eps:
        x_p = x + v * dt * np.cos(th)
        y_p = y + v * dt * np.sin(th)
        th_p = th  # straight motion, no rotation
    else:
        v_over_w = v / w
        th_dt = th + w * dt
        x_p = x - v_over_w * np.sin(th) + v_over_w * np.sin(th_dt)
        y_p = y + v_over_w * np.cos(th) - v_over_w * np.cos(th_dt)
        th_p = th_dt
    return (x_p, y_p, wrap_angle(th_p))

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

# --- Simulation Parameters ---
initial_pose = (0.0, 0.0, np.pi / 4)  # Start at (0,0) facing 45 degrees
v, w, dt = 1.0, 0.5, 0.1             # v=1 m/s, w=0.5 rad/s, dt=0.1 sec
total_time = 5.0                     # Simulate for 5 seconds
robot_radius = 0.25                  # Radius for plotting

# --- Run Simulation ---
poses = [initial_pose]
current_pose = initial_pose
num_steps = int(total_time / dt)

for _ in range(num_steps):
    current_pose = motion_model_ideal(current_pose, v, w, dt)
    poses.append(current_pose)

# --- Plot Results ---
poses = np.array(poses)
final_pose = poses[-1]

plt.figure(figsize=(10, 8), dpi=100)
ax = plt.gca()

# Plot the path
plt.plot(poses[:, 0], poses[:, 1], 'b--', label='Robot Path (x, y)')

# Plot the start and end poses
plot_robot_pose(ax, initial_pose, 'Start Pose', 'g', robot_radius)
plot_robot_pose(ax, final_pose, 'End Pose', 'r', robot_radius)

plt.title('Q2(a): Ideal Velocity Motion Model Path', fontsize=16)
plt.xlabel('X position (m)', fontsize=12)
plt.ylabel('Y position (m)', fontsize=12)
plt.legend()
plt.grid(True, linestyle=':', alpha=0.6)
plt.axis('equal')
plt.show()