import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class RRRRobotSimulator:
    def __init__(self, ax, target_rate, control_rate, label, with_obstacle=True):
        self.ax = ax
        self.target_rate = target_rate
        self.control_rate = control_rate
        self.label = label
        self.with_obstacle = with_obstacle

        self.L1 = self.L2 = self.L3 = 1.0
        self.T = 5
        self.f = 5
        self.dt = 1.0 / control_rate
        self.obstacle_center = np.array([2.0, 0.35])
        self.obstacle_radius = self.L1 / 8

        self._setup_trajectory()
        self._initialize_states()
        self._run_control_loop()
        self.ani = self._create_animation()

    def _obstacle_repulsion(self, x, y, strength=0.25):
        diff = np.array([x, y]) - self.obstacle_center
        dist = np.linalg.norm(diff)
        activation_radius = 2.0 * self.obstacle_radius
        if dist < activation_radius:
            return strength * ((activation_radius - dist) / activation_radius) * (diff / dist)
        return np.zeros(2)

    def _setup_trajectory(self):
        self.target_times = np.linspace(0, self.T, int(self.T * self.target_rate))
        self.control_times = np.linspace(0, self.T, int(self.T * self.control_rate))

        self.X_target = 2 * self.L1 * np.ones_like(self.target_times)
        self.Y_target = self.L1 * np.sin(2 * np.pi * self.f * self.target_times)

        self.X_control = np.zeros_like(self.control_times)
        self.Y_control = np.zeros_like(self.control_times)
        hold_steps = int(2.0 * self.control_rate)
        self.X_control = np.concatenate([np.full(hold_steps, 0), self.X_control])
        self.Y_control = np.concatenate([np.full(hold_steps, 0), self.Y_control])
        self.control_times = np.linspace(0, self.T + 2.0, len(self.X_control))

        last_idx = 0
        for i, t in enumerate(self.control_times):
            if t >= self.target_times[last_idx] and last_idx < len(self.target_times) - 1:
                last_idx += 1
            self.X_control[i] = self.X_target[last_idx - 1]
            self.Y_control[i] = self.Y_target[last_idx - 1]

    def _ik_rrr_all(self, x, y):
        r = np.hypot(x, y)
        if r > (self.L1 + self.L2 + self.L3):
            return []
        x2 = x - self.L3 * (x / r)
        y2 = y - self.L3 * (y / r)
        cos_theta2 = (x2**2 + y2**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        theta2_options = [np.arccos(cos_theta2), -np.arccos(cos_theta2)]
        solutions = []
        for theta2 in theta2_options:
            k1 = self.L1 + self.L2 * np.cos(theta2)
            k2 = self.L2 * np.sin(theta2)
            theta1 = np.arctan2(y2, x2) - np.arctan2(k2, k1)
            theta3 = np.arctan2(y - y2, x - x2) - (theta1 + theta2)
            solutions.append(np.array([theta1, theta2, theta3]))
        return solutions

    def _initialize_states(self):
        self.theta_current = np.radians([-160, 90, 120])
        self.theta_integral = np.zeros(3)
        self.theta_prev_error = np.zeros(3)
        self.theta_history = []
        self.ee_x = []
        self.ee_y = []

    def _run_control_loop(self):
        for i in range(len(self.X_control)):
            raw_target = np.array([self.X_control[i], self.Y_control[i]])
            repulsion = self._obstacle_repulsion(*raw_target) if self.with_obstacle else np.zeros(2)
            corrected_target = raw_target + repulsion

            t1, t2, t3 = self.theta_current
            x1 = self.L1 * np.cos(t1)
            y1 = self.L1 * np.sin(t1)
            x2 = x1 + self.L2 * np.cos(t1 + t2)
            y2 = y1 + self.L2 * np.sin(t1 + t2)
            x3 = x2 + self.L3 * np.cos(t1 + t2 + t3)
            y3 = y2 + self.L3 * np.sin(t1 + t2 + t3)
            ee_pos = np.array([x3, y3])

            if i == 0:
                theta_desired = self.theta_current.copy()
            else:
                ik_solutions = self._ik_rrr_all(*corrected_target)
                if not ik_solutions:
                    continue
                theta_desired = min(ik_solutions, key=lambda sol: np.linalg.norm(sol - self.theta_current))

            error = theta_desired - self.theta_current
            self.theta_integral += error * self.dt
            self.theta_integral = np.clip(self.theta_integral, -0.5, 0.5)
            theta_derivative = (error - self.theta_prev_error) / self.dt
            self.theta_current += 0.08 * error + 0.00001 * self.theta_integral + 0.00002 * theta_derivative
            self.theta_prev_error = error

            if self.with_obstacle:
                repulsion = self._obstacle_repulsion(*ee_pos)
                if np.linalg.norm(repulsion) > 0:
                    corrected_pos = ee_pos + repulsion
                    ik_repel_solutions = self._ik_rrr_all(*corrected_pos)
                    if ik_repel_solutions:
                        theta_repulsed = min(ik_repel_solutions, key=lambda sol: np.linalg.norm(sol - self.theta_current))
                        self.theta_current = 0.8 * self.theta_current + 0.2 * theta_repulsed

            self.theta_history.append(self.theta_current.copy())
            self.ee_x.append(x3)
            self.ee_y.append(y3)

    def _create_animation(self):
        self.target_dot, = self.ax.plot([], [], 'go', markersize=5)
        self.ee_dot, = self.ax.plot([], [], 'bo', markersize=5)
        self.line, = self.ax.plot([], [], 'k-', lw=1.5)
        self.path, = self.ax.plot([], [], 'b--', alpha=0.3)

        if self.with_obstacle:
            circle = plt.Circle(self.obstacle_center, self.obstacle_radius, color='r', alpha=0.3)
            self.ax.add_patch(circle)

        self.ax.set_xlim(-1, 4)
        self.ax.set_ylim(-2, 2)
        self.ax.set_aspect('equal')
        self.ax.set_title(self.label)
        self.ax.plot(self.X_control, self.Y_control, 'g:', linewidth=1.5, label='Target Trajectory')
        self.ax.grid(True)

        return animation.FuncAnimation(
            self.ax.figure,
            self._update_frame,
            frames=len(self.X_control),
            init_func=self._init_animation,
            blit=True,
            interval=1000 / self.control_rate
        )

    def _init_animation(self):
        self.target_dot.set_data([], [])
        self.ee_dot.set_data([], [])
        self.line.set_data([], [])
        self.path.set_data([], [])
        return self.target_dot, self.ee_dot, self.line, self.path

    def _update_frame(self, frame):
        theta = self.theta_history[frame]
        x1 = self.L1 * np.cos(theta[0])
        y1 = self.L1 * np.sin(theta[0])
        x2 = x1 + self.L2 * np.cos(theta[0] + theta[1])
        y2 = y1 + self.L2 * np.sin(theta[0] + theta[1])
        x3 = x2 + self.L3 * np.cos(theta[0] + theta[1] + theta[2])
        y3 = y2 + self.L3 * np.sin(theta[0] + theta[1] + theta[2])
        self.target_dot.set_data([self.X_control[frame]], [self.Y_control[frame]])
        self.ee_dot.set_data([self.ee_x[frame]], [self.ee_y[frame]])
        self.line.set_data([0, x1, x2, x3], [0, y1, y2, y3])
        self.path.set_data(self.ee_x[:frame], self.ee_y[:frame])
        return self.target_dot, self.ee_dot, self.line, self.path

    def get_animation(self):
        return self.ani
