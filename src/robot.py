import numpy as np
import matplotlib
# matplotlib.use('TkAgg')
try:
    matplotlib.use('TkAgg')
except ImportError:
    print("⚠️ TkAgg not available, falling back to Agg.")
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# Robot Parameters
# --------------------------
L1 = 1.0
L2 = 1.0
L3 = 1.0  # Length of each link

# --------------------------
# Simulation Parameters
# --------------------------
f = 5                 # Frequency of target movement in Hz
target_rate = 5       # Target update frequency in Hz
control_rate = 50     # Robot control frequency in Hz
T = 5                 # Total simulation time in seconds
Kp = 0.5              # Proportional gain for joint update

# --------------------------
# Time Vectors
# --------------------------
target_times = np.linspace(0, T, int(T * target_rate))
control_times = np.linspace(0, T, int(T * control_rate))

# --------------------------
# Target Trajectory (sinusoidal wave)
# --------------------------

X_target = 2 * L1 * np.ones_like(target_times)  # constant x = 2L
Y_target = L1 * np.sin(2 * np.pi * f * target_times)  # y = L·sin(2πft)


# --------------------------
# Obstacle definition
# --------------------------
obstacle_center = np.array([2.0, 0.35])
obstacle_radius = L1 / 8

def obstacle_repulsion(x, y, strength=0.25):
    diff = np.array([x, y]) - obstacle_center
    dist = np.linalg.norm(diff)
    activation_radius = 2.0 * obstacle_radius
    if dist < activation_radius:
        return strength * ((activation_radius - dist) / activation_radius) * (diff / dist)
    return np.zeros(2)

# --------------------------
# Zero-Order Hold Target (no interpolation)
# --------------------------
X_control = np.zeros_like(control_times)
Y_control = np.zeros_like(control_times)
hold_steps = int(2.0 * control_rate)  # 2 seconds hold duration

# Hold the first target value for `hold_steps` samples
X_control = np.concatenate([np.full(hold_steps, X_control[0]), X_control])
Y_control = np.concatenate([np.full(hold_steps, Y_control[0]), Y_control])

# Pad other vectors accordingly
control_times = np.linspace(0, T + 2.0, len(X_control))

last_idx = 0
for i, t in enumerate(control_times):
    if t >= target_times[last_idx] and last_idx < len(target_times) - 1:
        last_idx += 1
    X_control[i] = X_target[last_idx - 1]
    Y_control[i] = Y_target[last_idx - 1]

# --------------------------
# Inverse Kinematics Solver (returns both solutions)
# --------------------------
def ik_rrr_all(x, y):
    r = np.hypot(x, y)
    if r > (L1 + L2 + L3):
        return []  # unreachable

    x2 = x - L3 * (x / r)
    y2 = y - L3 * (y / r)

    cos_theta2 = (x2**2 + y2**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2_options = [np.arccos(cos_theta2), -np.arccos(cos_theta2)]
    solutions = []

    for theta2 in theta2_options:
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y2, x2) - np.arctan2(k2, k1)
        theta3 = np.arctan2(y - y2, x - x2) - (theta1 + theta2)
        solutions.append(np.array([theta1, theta2, theta3]))

    return solutions

# --------------------------
# Initial Position
# --------------------------
start_pos = np.array([0.5, -0.5])
theta_start = np.array(ik_rrr_all(*start_pos)[0])  # just pick one

# --------------------------
# Forward Kinematics Solver
# --------------------------
def set_initial_pose(theta1, theta2, theta3, hold_duration=2.0):
    theta_current = np.array([theta1, theta2, theta3])
    t1, t2, t3 = theta_current

    # Forward kinematics to compute EE
    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)
    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)
    x3 = x2 + L3 * np.cos(t1 + t2 + t3)
    y3 = y2 + L3 * np.sin(t1 + t2 + t3)

    return theta_current

# --------------------------
# PID Setup
# --------------------------
Kp_pid = 0.08
Ki_pid = 0.00001
Kd_pid = 0.00002
dt = 1.0 / control_rate

theta_current = set_initial_pose(np.radians(-160), np.radians(90), np.radians(120), hold_duration=0.0)
theta_integral = np.zeros(3)
theta_prev_error = np.zeros(3)

theta_history = []
ee_x, ee_y = [], []
target_log = []

# --------------------------
# Control Loop
# --------------------------
for i in range(len(X_control)):
    raw_target = np.array([X_control[i], Y_control[i]])
    repulsion = obstacle_repulsion(*raw_target)
    corrected_target = raw_target + repulsion
    target_log.append({
        'Time (s)': control_times[i],
        'Raw_X': raw_target[0],
        'Raw_Y': raw_target[1],
        'Corrected_X': corrected_target[0],
        'Corrected_Y': corrected_target[1],
    })
    t1, t2, t3 = theta_current
    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)
    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)
    x3 = x2 + L3 * np.cos(t1 + t2 + t3)
    y3 = y2 + L3 * np.sin(t1 + t2 + t3)
    ee_pos = np.array([x3, y3])

    if i == 0:
        theta_desired = theta_current.copy()
    else:
        ik_solutions = ik_rrr_all(*corrected_target)
        if not ik_solutions:
            continue
        theta_desired = min(ik_solutions, key=lambda sol: np.linalg.norm(sol - theta_current))

    error = theta_desired - theta_current
    theta_integral += error * dt
    theta_integral = np.clip(theta_integral, -0.5, 0.5)
    theta_derivative = (error - theta_prev_error) / dt
    theta_current += (
        Kp_pid * error +
        Ki_pid * theta_integral +
        Kd_pid * theta_derivative
    )
    theta_prev_error = error

    repulsion = obstacle_repulsion(*ee_pos)
    if np.linalg.norm(repulsion) > 0:
        corrected_pos = ee_pos + repulsion
        ik_repel_solutions = ik_rrr_all(*corrected_pos)
        if ik_repel_solutions:
            theta_repulsed = min(ik_repel_solutions, key=lambda sol: np.linalg.norm(sol - theta_current))
            theta_current = 0.8 * theta_current + 0.2 * theta_repulsed

    theta_history.append(theta_current.copy())
    ee_x.append(x3)
    ee_y.append(y3)

# df = pd.DataFrame(target_log)
# df.to_excel("target_vs_corrected.xlsx", index=False)
# --------------------------
# Visualization Setup
# --------------------------
fig, ax = plt.subplots()
ax.set_xlim(-1, 4)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid(True)

target_dot, = ax.plot([], [], 'go', markersize=8, label='Target')
ee_dot, = ax.plot([], [], 'bo', label='End Effector')
line, = ax.plot([], [], 'k-', lw=2)
path, = ax.plot([], [], 'b--', alpha=0.3)
circle = plt.Circle(obstacle_center, obstacle_radius, color='r', alpha=0.3)
ax.add_patch(circle)
repulsion_arrows = []

# --------------------------
# Animation Init and Update
# --------------------------
def init():
    target_dot.set_data([], [])
    ee_dot.set_data([], [])
    line.set_data([], [])
    path.set_data([], [])
    return target_dot, ee_dot, line, path

def update(frame):
    x_t, y_t = X_control[frame], Y_control[frame]
    x_e, y_e = ee_x[frame], ee_y[frame]
    theta = theta_history[frame]

    x1 = L1 * np.cos(theta[0])
    y1 = L1 * np.sin(theta[0])
    x2 = x1 + L2 * np.cos(theta[0] + theta[1])
    y2 = y1 + L2 * np.sin(theta[0] + theta[1])
    x3 = x2 + L3 * np.cos(theta[0] + theta[1] + theta[2])
    y3 = y2 + L3 * np.sin(theta[0] + theta[1] + theta[2])

    target_dot.set_data(x_t, y_t)
    ee_dot.set_data(x_e, y_e)
    line.set_data([0, x1, x2, x3], [0, y1, y2, y3])
    path.set_data(ee_x[:frame], ee_y[:frame])

    for arrow in repulsion_arrows:
        arrow.remove()
    repulsion_arrows.clear()
    rep_vec = obstacle_repulsion(x_t, y_t)
    if np.linalg.norm(rep_vec) > 0:
        arrow = ax.arrow(x_t, y_t, rep_vec[0], rep_vec[1], head_width=0.05, color='orange')
        repulsion_arrows.append(arrow)

    return target_dot, ee_dot, line, path, *repulsion_arrows

# --------------------------
# Run Animation
# --------------------------
ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(X_control),
    init_func=init,
    blit=True,
    interval=1000 / control_rate
)
# ani.save("rrr_animation.mp4", writer="ffmpeg", fps=30)
ax.plot(X_control, Y_control, 'g:', linewidth=1.5, label='Target Trajectory')
plt.title("3-DOF RRR Robot with Gradual Motion Tracking a Target")
plt.legend()
plt.tight_layout()
plt.show()
