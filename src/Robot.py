import argparse
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from sim import RRRRobotSimulator

parser = argparse.ArgumentParser()
parser.add_argument('--obstacle', action='store_true', help="Include obstacle in the simulation.")
args = parser.parse_args()

if args.obstacle:
    fig1, axs1 = plt.subplots(1, 2, figsize=(14, 6))
    (ax1, ax2) = axs1
    ani1 = RRRRobotSimulator(ax1, target_rate=5, control_rate=50, label="With Obstacle: 5Hz / 50Hz", with_obstacle=True)
    ani2 = RRRRobotSimulator(ax2, target_rate=30, control_rate=1000, label="With Obstacle: 30Hz / 1000Hz", with_obstacle=True)
    fig1.suptitle("Simulations With Obstacle", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  
    plt.show()
else:
    fig2, axs2 = plt.subplots(1, 2, figsize=(14, 6))
    (ax3, ax4) = axs2
    ani3 = RRRRobotSimulator(ax3, target_rate=5, control_rate=50, label="No Obstacle: 5Hz / 50Hz", with_obstacle=False)
    ani4 = RRRRobotSimulator(ax4, target_rate=30, control_rate=1000, label="No Obstacle: 30Hz / 1000Hz", with_obstacle=False)
    fig2.suptitle("Simulations Without Obstacle", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()