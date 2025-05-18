# Robot End-Effector Control (RRR Planar Arm Simulation)

This project simulates a 3-DOF planar RRR (Revolute-Revolute-Revolute) robot arm tracking a moving target in 2D space. It includes inverse kinematics, PID control, and optional obstacle avoidance using repulsion. Animations are rendered using `matplotlib`.

---

## Project Structure

```
Robot_EndEffector_Control/
├── docker-compose.yml         # Docker Compose for running simulation
├── Dockerfile                 # Python-based simulation image
├── requirements.txt           # Python dependencies
├── README.md                  # Project documentation
└── src/
    └── Robot.py               # Main simulation script
```

---

## Features

- 2D Cartesian tracking of a moving sinusoidal target
- Inverse kinematics solver for RRR manipulator
- PID-based joint control with integral and derivative terms
- Obstacle repulsion modeled using a virtual force field
- Real-time animation of robot arm and end-effector path

---

## Running the Simulation

### Prerequisites

- Ensure [Docker](https://www.docker.com/) is installed and running.
- Docker compose is installed

### Run with obstacle:

```bash
docker compose --profile sim-obstacle up --build
```
### Run without obstacle:

```bash
docker compose --profile sim up --build
```

Each launches a side-by-side animation of the robot behavior at:
- 5 Hz target / 50 Hz control rate
- 30 Hz target / 1000 Hz control rate


