# Quadruped Robot Motion Control

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a%2B-orange) ![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-V--REP-blue) ![Branch](https://img.shields.io/badge/branch-motion--control-green)

This repository contains the kinematic modeling, trajectory planning, and simulation control algorithms for a quadruped robot dog. The project connects MATLAB algorithms with a CoppeliaSim (V-REP) physics environment to simulate gait patterns and leg movements.

> **Current Branch:** `motion-control`
> This branch focuses on the implementation of Inverse Kinematics (IK), trajectory generation for leg swings, and real-time communication with the simulation environment.

## Features

* **Kinematic Modeling:**
    * **Forward Kinematics (FK):** Calculates foot tip positions based on joint angles using geometric transformation matrices. See [ForwardKinematics.m](./ForwardKinematics.m).
    * **Inverse Kinematics (IK):** Solves required joint angles ($\theta_1, \theta_2, \theta_3$) for given cartesian coordinates $(x, y, z)$ relative to the leg base. See [InverseKinematics.m](./InverseKinematics.m).
    * **Workspace Analysis:** Visualizes the reachability cloud of a single leg. See [WorkSpace.m](./WorkSpace.m).
* **Motion Planning:**
    * **Trajectory Generation:** Creates rectangular path waypoints (points A $\to$ B $\to$ C $\to$ D) for foot trajectory. See [TrajectoryPlanning.m](./TrajectoryPlanning.m).
    * **Gait Logic:** Implements gait state machines (e.g., `ZERO` stance, `FORWARD` trot). See [Gait.m](./Gait.m).
* **Simulation Integration:**
    * Connects to CoppeliaSim via the Legacy Remote API.
    * Synchronous joint control for all 4 legs (LF, LB, RF, RB). See [test.m](./test.m).

## Prerequisites

To run this code, you need:

1.  **MATLAB** (Tested on recent versions).
2.  **Robotics Toolbox for MATLAB** (by Peter Corke): Required for `Link` and `SerialLink` functions used in [InitModel.m](./InitModel.m).
3.  **CoppeliaSim (formerly V-REP):** For the 3D simulation environment.

## Project Structure

| File | Description |
| :--- | :--- |
| [main.m](./main.m) | Entry point for testing single-leg trajectories and kinematics without full simulation. |
| [test.m](./test.m) | **Main Simulation Loop.** Connects to CoppeliaSim, initializes joint handles, and streams gait trajectories to the robot. |
| [ForwardKinematics.m](./ForwardKinematics.m) | Computes end-effector position given joint angles and leg configuration (LF, LB, RF, RB). |
| [InverseKinematics.m](./InverseKinematics.m) | Computes joint angles given a target coordinate $(x,y,z)$. |
| [Gait.m](./Gait.m) | Generates the sequence of joint angles (`theta_i`) required for specific movements (Forward, Zero). |
| [TrajectoryPlanning.m](./TrajectoryPlanning.m) | Generates Cartesian waypoints for the foot's swing phase and visualizes the path. |
| [InitModel.m](./InitModel.m) | Initializes the `SerialLink` object for a specific leg using DH parameters. |
| [remoteApiProto.m](./remoteApiProto.m) | Prototype file for the CoppeliaSim remote API. |

## Setup & Usage

### 1. Simulation Setup
1.  Open **CoppeliaSim**.
2.  Load your quadruped robot scene.
3.  Ensure the remote API server is running (usually started automatically in the scene scripts or via `remoteApiStart(19999)`).

### 2. Running the Control Loop
To control the robot in the simulation:

1.  Open `test.m` in MATLAB.
2.  Ensure `remoteApiProto.m`, `remApi.m`, and the appropriate `.dll/.so/.dylib` library for the remote API are in your path.
3.  Run the script:
    ```matlab
    >> test
    ```
4.  The script will:
    * Connect to the API server on port `19999`.
    * Retrieve handles for joints `Revolute_joint_xx`.
    * Execute the `FORWARD` gait loop.

### 3. Testing Kinematics Only
To verify the math without opening the simulator:

1.  Open `main.m`.
2.  Adjust `robot_config.leg_type` (e.g., `"left-front"`).
3.  Run `main.m` to plot the leg model and calculated trajectory.

## Configuration

Robot physical parameters are defined in the `robot_length` structure within `main.m` and `test.m`:

```matlab
robot_length = struct('base_length', 0.5, ...
                      'base_width', 0.2, ...
                      'L1', 0.05, ... % Hip length
                      'L2', 0.15, ... % Upper leg length
                      'L3', 0.1);     % Lower leg length
