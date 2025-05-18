import numpy as np
import matplotlib
matplotlib.use('TkAgg')
# try:
#     matplotlib.use('TkAgg')
# except ImportError:
#     print("⚠️ TkAgg not available, falling back to Agg.")
#     matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import argparse

def run_simulation(ax, target_rate, control_rate, label, with_obstacle=True):
    T = 5
    f = 5
    L1 = L2 = L3 = 1.0
    obstacle_center = np.array([2.0, 0.35])
    obstacle_radius = L1 / 8

    def obstacle_repulsion(x, y, strength=0.25):
        diff = np.array([x, y]) - obstacle_center
        dist = np.linalg.norm(diff)
        activation_radius = 2.0 * obstacle_radius
        if dist < activation_radius:
            return strength * ((activation_radius - dist) / activation_radius) * (diff / dist)
        return np.zeros(2)

    target_times = np.linspace(0, T, int(T * target_rate))
    control_times = np.linspace(0, T, int(T * control_rate))
    X_target = 2 * L1 * np.ones_like(target_times)
    Y_target = L1 * np.sin(2 * np.pi * f * target_times)

    X_control = np.zeros_like(control_times)
    Y_control = np.zeros_like(control_times)
    hold_steps = int(2.0 * control_rate)
    X_control = np.concatenate([np.full(hold_steps, X_control[0]), X_control])
    Y_control = np.concatenate([np.full(hold_steps, Y_control[0]), Y_control])
    control_times = np.linspace(0, T + 2.0, len(X_control))
    last_idx = 0
    for i, t in enumerate(control_times):
        if t >= target_times[last_idx] and last_idx < len(target_times) - 1:
            last_idx += 1
        X_control[i] = X_target[last_idx - 1]
        Y_control[i] = Y_target[last_idx - 1]

    def ik_rrr_all(x, y):
        r = np.hypot(x, y)
        if r > (L1 + L2 + L3):
            return []
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

    def set_initial_pose(t1, t2, t3):
        return np.array([t1, t2, t3])

    theta_current = set_initial_pose(np.radians(-160), np.radians(90), np.radians(120))
    theta_integral = np.zeros(3)
    theta_prev_error = np.zeros(3)
    dt = 1.0 / control_rate
    theta_history = []
    ee_x, ee_y = [], []

    for i in range(len(X_control)):
        raw_target = np.array([X_control[i], Y_control[i]])
        repulsion = obstacle_repulsion(*raw_target) if with_obstacle else np.zeros(2)
        corrected_target = raw_target + repulsion
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
        theta_current += 0.08 * error + 0.00001 * theta_integral + 0.00002 * theta_derivative
        theta_prev_error = error
        repulsion = obstacle_repulsion(*ee_pos) if with_obstacle else np.zeros(2)
        if np.linalg.norm(repulsion) > 0:
            corrected_pos = ee_pos + repulsion
            ik_repel_solutions = ik_rrr_all(*corrected_pos)
            if ik_repel_solutions:
                theta_repulsed = min(ik_repel_solutions, key=lambda sol: np.linalg.norm(sol - theta_current))
                theta_current = 0.8 * theta_current + 0.2 * theta_repulsed
        theta_history.append(theta_current.copy())
        ee_x.append(x3)
        ee_y.append(y3)

    target_dot, = ax.plot([], [], 'go', markersize=5)
    ee_dot, = ax.plot([], [], 'bo', markersize=5)
    line, = ax.plot([], [], 'k-', lw=1.5)
    path, = ax.plot([], [], 'b--', alpha=0.3)
    
    if with_obstacle:
        circle = plt.Circle(obstacle_center, obstacle_radius, color='r', alpha=0.3)
        ax.add_patch(circle)

    ax.set_xlim(-1, 4)
    ax.plot(X_control, Y_control, 'g:', linewidth=1.5, label='Target Trajectory')
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.set_title(label)
    ax.grid(True)

    def init():
        target_dot.set_data([], [])
        ee_dot.set_data([], [])
        line.set_data([], [])
        path.set_data([], [])
        return target_dot, ee_dot, line, path

    def update(frame):
        theta = theta_history[frame]
        x1 = L1 * np.cos(theta[0])
        y1 = L1 * np.sin(theta[0])
        x2 = x1 + L2 * np.cos(theta[0] + theta[1])
        y2 = y1 + L2 * np.sin(theta[0] + theta[1])
        x3 = x2 + L3 * np.cos(theta[0] + theta[1] + theta[2])
        y3 = y2 + L3 * np.sin(theta[0] + theta[1] + theta[2])
        target_dot.set_data([X_control[frame]], [Y_control[frame]])
        ee_dot.set_data([ee_x[frame]], [ee_y[frame]])
        line.set_data([0, x1, x2, x3], [0, y1, y2, y3])
        path.set_data(ee_x[:frame], ee_y[:frame])
        return target_dot, ee_dot, line, path

    ani = animation.FuncAnimation(
        ax.figure,
        update,
        frames=len(X_control),
        init_func=init,
        blit=True,
        interval=1000 / control_rate
    )
    return ani

parser = argparse.ArgumentParser()
parser.add_argument('--obstacle', action='store_true', help="Include obstacle in the simulation.")
args = parser.parse_args()

if args.obstacle:
    fig1, axs1 = plt.subplots(1, 2, figsize=(14, 6))
    (ax1, ax2) = axs1
    ani1 = run_simulation(ax1, target_rate=5, control_rate=50, label="With Obstacle: 5Hz / 50Hz", with_obstacle=True)
    ani2 = run_simulation(ax2, target_rate=30, control_rate=1000, label="With Obstacle: 30Hz / 1000Hz", with_obstacle=True)
    fig1.suptitle("Simulations With Obstacle", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  
    plt.show()
else:
    fig2, axs2 = plt.subplots(1, 2, figsize=(14, 6))
    (ax3, ax4) = axs2
    ani3 = run_simulation(ax3, target_rate=5, control_rate=50, label="No Obstacle: 5Hz / 50Hz", with_obstacle=False)
    ani4 = run_simulation(ax4, target_rate=30, control_rate=1000, label="No Obstacle: 30Hz / 1000Hz", with_obstacle=False)
    fig2.suptitle("Simulations Without Obstacle", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()



