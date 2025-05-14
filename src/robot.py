import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


L1 = L2 = L3 = 1.0  
f = 5                
target_rate = 5       
control_rate = 50     
T = 5                
Kp = 0.5              

target_times = np.linspace(0, T, int(T * target_rate))
control_times = np.linspace(0, T, int(T * control_rate))

X_target = 1.5 + 1.0 * target_times / T 
Y_target = 0.5 * np.sin(2 * np.pi * f * target_times)

X_control = np.zeros_like(control_times)
Y_control = np.zeros_like(control_times)
last_idx = 0
for i, t in enumerate(control_times):
    if t >= target_times[last_idx] and last_idx < len(target_times) - 1:
        last_idx += 1
    X_control[i] = X_target[last_idx - 1]
    Y_control[i] = Y_target[last_idx - 1]

def ik_rrr(x, y):
    r = np.hypot(x, y)
    if r > (L1 + L2 + L3):
        return [0, 0, 0]  
    x2 = x - L3 * (x / r)
    y2 = y - L3 * (y / r)
    cos_theta2 = (x2**2 + y2**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y2, x2) - np.arctan2(k2, k1)
    theta3 = np.arctan2(y - y2, x - x2) - (theta1 + theta2)
    return [theta1, theta2, theta3]

theta_current = np.array(ik_rrr(X_control[0], Y_control[0]))
ee_x, ee_y = [], []
theta_history = []

for x_t, y_t in zip(X_control, Y_control):
    theta_desired = np.array(ik_rrr(x_t, y_t))
    theta_current += Kp * (theta_desired - theta_current)
    theta_history.append(theta_current.copy())

    x1 = L1 * np.cos(theta_current[0])
    y1 = L1 * np.sin(theta_current[0])
    x2 = x1 + L2 * np.cos(theta_current[0] + theta_current[1])
    y2 = y1 + L2 * np.sin(theta_current[0] + theta_current[1])
    x3 = x2 + L3 * np.cos(theta_current[0] + theta_current[1] + theta_current[2])
    y3 = y2 + L3 * np.sin(theta_current[0] + theta_current[1] + theta_current[2])
    ee_x.append(x3)
    ee_y.append(y3)

fig, ax = plt.subplots()
ax.set_xlim(-1, 4)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid(True)

target_dot, = ax.plot([], [], 'go', markersize=8, label='Target')
ee_dot, = ax.plot([], [], 'bo', label='End Effector')
line, = ax.plot([], [], 'k-', lw=2)
path, = ax.plot([], [], 'b--', alpha=0.3)

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
    return target_dot, ee_dot, line, path

ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(control_times),
    init_func=init,
    blit=True,
    interval=1000 / control_rate 
)

plt.title("3-DOF RRR Robot with Gradual Motion Tracking a Target")
plt.legend()
plt.tight_layout()
plt.show()
