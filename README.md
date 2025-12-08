# Quadruped Robot Control Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange.svg)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-Edu-blue.svg)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow)

**A MATLAB-based control framework for a quadruped robot dog, utilizing CoppeliaSim (V-REP) for physics simulation via the Remote API.**

## Project Overview

This project implements the kinematic modeling and gait planning algorithms for a 12-DOF quadruped robot. The control logic is written entirely in MATLAB, which computes joint angles and trajectory points. These commands are sent in real-time to a CoppeliaSim simulation environment using the legacy Remote API.

The project features:
* **Custom Kinematics:** Analytical solutions for Forward (FK) and Inverse Kinematics (IK).
* **Visualization:** MATLAB-based stick-figure visualization using the Robotics Toolbox.
* **Gait Planning:** Generation of joint-space trajectories for various movements (Forward, Zero position).
* **Simulation Loop:** Synchronous communication with the physics engine.

## Project Structure

| File | Description |
| :--- | :--- |
| `main.m` | **Entry point.** Initializes the robot, plots the workspace, and triggers trajectory planning. |
| `InitModel.m` | Defines the DH parameters and link structures (requires Robotics Toolbox). |
| `ForwardKinematics.m`| Calculates end-effector position $(x,y,z)$ given joint angles $(\theta_1, \theta_2, \theta_3)$. |
| `InverseKinematics.m`| Calculates required joint angles given a target coordinate. |
| `Gait.m` | Generates trajectory waypoints for specific movements (Forward, Turn, etc.). |
| `remApi.m` | MATLAB wrapper for the CoppeliaSim Legacy Remote API. |
| `remoteApiProto.m` | Prototype file defining library function signatures. |
| `remoteApi.dll` | Windows Dynamic Link Library for the Remote API. |

## Prerequisites

To run this project, you need:

1.  **MATLAB** (Tested on version R202x).
2.  **CoppeliaSim** (formerly V-REP).
3.  **Robotics Toolbox for MATLAB** (by Peter Corke) - Used in `InitModel.m` for visualization.
    * *Installation:* `RTB` via MATLAB Add-On Explorer.
4.  **C Compiler** (MinGW recommended) - Required to load the `remoteApi` library in MATLAB.

## Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/your-username/coppeliasim-quadruped-control.git](https://github.com/your-username/coppeliasim-quadruped-control.git)
    ```
2.  **Setup CoppeliaSim:**
    * Open CoppeliaSim.
    * Load your robot scene (`.ttt` file).
    * Ensure there is a child script in the scene enabling the remote API (usually on port 19999).
3.  **Setup MATLAB:**
    * Open MATLAB and navigate to the project folder.
    * Ensure `remoteApi.dll` (Windows), `.so` (Linux), or `.dylib` (Mac) matches your system architecture (64-bit).

## Usage

1.  Start the simulation in CoppeliaSim.
2.  Run the test script in MATLAB:
    ```matlab
    test
    ```
3.  The script will:
    * Initialize the robot parameters (Link lengths: $L_1=0.05, L_2=0.15, L_3=0.1$).
    * Plot the static model in a MATLAB figure.
    * Calculate trajectories via `TrajectoryPlanning`.
    * (Future) Send commands to CoppeliaSim.

## Kinematics

The robot legs are modeled using a 3-DOF configuration (Hip, Upper Leg, Lower Leg).

* **Inverse Kinematics:** Solved analytically using geometric decomposition to determine $\theta_1, \theta_2, \theta_3$.
* **Coordinate Frames:** Handled dynamically for different leg positions (Left-Front, Right-Behind, etc.) within `InverseKinematics.m`.

## Contributing

Contributions are welcome! Please open an issue if you encounter bugs with the Remote API connection or kinematic singularities.
