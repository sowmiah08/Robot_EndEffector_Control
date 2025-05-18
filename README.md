# Robot End-Effector Control

This project simulates a 3-DOF planar RRR (Revolute-Revolute-Revolute) robot arm tracking a moving target in 2D space. It includes inverse kinematics, PID control and optional obstacle avoidance using repulsion force field. Animations are created using `matplotlib`.

---

## Directory Structure

```
Robot_EndEffector_Control/
├── docker-compose.yml                                      # Docker Compose for running simulation
├── Intern Challenge_ RRR Robot End-Effector Control.pdf    # Internship - Question
├── Dockerfile                                              # Python-based simulation image
├── requirements.txt                                        # Python dependencies
├── README.md                                               # Project documentation
└── src/
    ├── Robot.py                                            # Main script
    └── sim.py                                              # Simulator Class
```

---

## Dependencies and System Requirements

Python Requirements:
- numpy 
- scipy 
- matplotlib
- pandas 
- openpyxl 

All the requirements are installed inside the docker container. Hence no local installation is required for this to run. 

Tested On:
- OS : Ubuntu 24.04
- IDE: VS Code
- GPU: RTX 5090
- CPU: AMD 9950X3D
- RAM: 96GB RAM

---

## Implementations

- 2D Cartesian tracking of a moving sinusoidal target
- Inverse kinematics solver for RRR manipulator
- PID-based joint control with integral and derivative terms
- Obstacle repulsion modeled using a virtual force field
- Real-time animation of robot arm and end-effector path

---

## Setup and Run the Code

### Prerequisites

- Ensure docker and docker-compose is installed locally.

### Run with obstacle:

```bash
docker compose --profile sim-obstacle up --build
```
### Run without obstacle:

```bash
docker compose --profile sim up --build
```

Each code launches a side-by-side animation of the robot behavior at:
- 5 Hz target / 50 Hz control rate
- 30 Hz target / 1000 Hz control rate

---

## Description of Our Approach

Assumptions:
- L1 = L2 = L3 = L = 1 (Considering the leghts of all links as same and equal)
- Obstacle Centre = [2.0, 0.35]
- Obstacle Radius = 0.125 




This simulation tracks the trajectory of a Cartesian trajectory of a 3DOF Planar Robot manipulator following a moving target in a 2D space. The target moves in a frequency giving a sequence of end effector positions the robot should move for following. The approaches used to accomplish this task are inverse kinematics and a basic PID joint controller. 

The trajectory of the robot and target is defined by a function **_setup_trajectory()**. The target trajectory is a sinusoidal path along the Y-axis while keeping the X-axis constant at a value of **2*L** . The robot set to follow the target is held two seconds in the beginning for an initial delay (to follow the target instead of overlapping it). By checking the most recent available target position and assigning target's position coordinates (x, y) to the robot control trajectory, the path the robot should move is defined. This allows the robot to track and follow the target with high precision and minimal delays. The joint configuration of the robot arm is calculated by the Inverse Kinematics (IK) method as the moving target provides us with the target position for the end-effector to reach. Also the Task A given for challenge clearly specifies that the RRR robot’s end effector should track the green target using only the target position data, therefore inverse kinematics is an appropriate approach. The **_ik_rrr_all()** function computes the possible joint configurations using closed-form solutions with the given target end effector position (x, y). Trigonometric functions produce more than one solution for the joint configurations. We do not want the robot movements to be clumsy or unsteady hence for minimizing the error over time and for smooth trajectory, PID  Proportional–Integral–Derivative control loops are used. The Proportional term used adjusts the current error at that instance, Integral term **theta_integral** accumulates the error over time for reducing the steady-state error and the Derivative controller **theta_derivative** tracks the rate of change of error. Thus PID helps prevent any overshoot or undershoot and ensures a smooth trajectory. The given task is actually completed using these functions where for **task A**the target is moved at a frequency of **30 Hz** and for **task B** the target position frequency is set to **5Hz** and control loop frequency to **50 Hz**. 

For the optional task, a **red circular obstacle** is introduced in the plane at (L, 0.5); where L is the length of the robot link. Now a repulsive force is created around the obstacle in order for the robot to avoid colliding with it. The repulsion function **_obstacle_repulsion()** modifies the target position by creating a safe zone (twice the obstacle radius) around the obstacle and if the target position is too close or in this zone it pushes the target position away from this boundary. The closer the robot gets the stronger push. This helps the end effector of the robot to achieve the obstacle avoidance. The visualization of the simulation is done using **matplotlib.animation**, showing the movement of the robot arm, target path, and end-effector trajectory. The visualization clearly shows how the robot arm moves as the end-effector(**blue point**) follows the moving target (**green point**) at different frequencies, it also shows the path/ trajectory of the end effector and target in**blue** and **green** dotted lines respectively. When an obstacle(ˇˇred circle) is included in the simulation, it can be observed how the robot movement is adjusted to avoid collision with the obstacle and its trajectory is shown.
   

Two simulation configurations are used here and the ouputs are given below in the Simulation Outputs section.
- **Without obstacle**: Simple target tracking using IK and PID.
- **With obstacle**: Dynamically adjusts motion based on virtual repulsion from the obstacle.

**NOTE**: 
- In the given Task, Target motion was given as (x, y) = (2L, L · sin(2πft)). Which means that X is constant and will travel in a straight line as seen from X-Axis. Y varies based on the X. But this does not look sinusoidal 
as seen from XY Plane. This is assumed that the platform must be moving and hence the sinusoidal wave.  
- The position for Obstacle was given as (x, y) = (L, 0.5L). This position according to out configuration is off the graph. The robot which follows  (x, y) = (2L, L · sin(2πft)) patgh never touches or interferes with the obstacle.
Hence a different Obstacle position was chosen to demonstarte the obstacle avoidance algorithm. The position assumed was (2.0, 0.35) for better Visualization. 


---

## Simulation Ouput

Below are outputs from the Simulations.

### 🔹 Without Obstacle vs With Obstacle

<p align="center">
  <img src="Output/sim.png" width="45%" alt="RRR Simulation without obstacle"/>
  <img src="Output/sim_obs.png" width="45%" alt="RRR Simulation with obstacle"/>
</p>

From the Figure, We can confirm the working of Obstacle avoidance. If an obstacle is detected, the imaginary force field will repel the End-Effector and thus avoiding the collision with the obstacle. 

---