# Quadruped Robot — Simulation and Control

> **Author:** LvDaengineer  
> **Project:** [quadruped-robot](https://github.com/users/longvotheengineer/projects/3/views/4)  
> **License:** MIT

---

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange.svg)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-4.6.0-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)
![Gazebo](https://img.shields.io/badge/Gazebo_Classic-11-red.svg)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple.svg)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow.svg)

A 12-DOF quadruped robot simulation and control framework. This project provides two independent simulation environments:

| Simulation | Engine | Control | Platform |
| :--- | :--- | :--- | :--- |
| **Part I** | CoppeliaSim (V-REP) | MATLAB via Legacy Remote API | Windows / Linux |
| **Part II** | Gazebo Classic 11 + ROS 2 Humble | Python (rclpy) | Linux (Ubuntu 22.04) |

Both simulations implement analytical inverse kinematics, gait generation, and trajectory planning for a quadruped robot with four 3-DOF legs.

---

## Table of Contents

- [Repository Structure](#repository-structure)
- [Branch Strategy](#branch-strategy)
- [Part I — CoppeliaSim + MATLAB Simulation](#part-i--coppeliasim--matlab-simulation)
  - [I.1 Overview](#i1-overview)
  - [I.2 Prerequisites](#i2-prerequisites)
  - [I.3 Installing CoppeliaSim](#i3-installing-coppeliasim)
  - [I.4 Installing MATLAB](#i4-installing-matlab)
  - [I.5 Clone the Repository](#i5-clone-the-repository)
  - [I.6 Project Structure (MATLAB)](#i6-project-structure-matlab)
  - [I.7 Setup and Configuration](#i7-setup-and-configuration)
  - [I.8 Running the Simulation](#i8-running-the-simulation)
  - [I.9 Expected Behavior](#i9-expected-behavior)
  - [I.10 Common Errors and Solutions](#i10-common-errors-and-solutions)
- [Part II — Gazebo + ROS 2 Simulation](#part-ii--gazebo--ros-2-simulation)
  - [II.1 Overview](#ii1-overview)
  - [II.2 Prerequisites](#ii2-prerequisites)
  - [II.3 Installing ROS 2 Humble](#ii3-installing-ros-2-humble)
  - [II.4 Installing Gazebo Classic 11](#ii4-installing-gazebo-classic-11)
  - [II.5 Installing Additional ROS 2 Dependencies](#ii5-installing-additional-ros-2-dependencies)
  - [II.6 Clone the Repository](#ii6-clone-the-repository)
  - [II.7 Project Structure (ROS 2)](#ii7-project-structure-ros-2)
  - [II.8 Building the Workspace](#ii8-building-the-workspace)
  - [II.9 Running the Simulation](#ii9-running-the-simulation)
  - [II.10 Publishing Gait Commands](#ii10-publishing-gait-commands)
  - [II.11 Expected Behavior](#ii11-expected-behavior)
  - [II.12 Common Errors and Solutions](#ii12-common-errors-and-solutions)
- [Contributing](#contributing)
- [Contact](#contact)

---

# Part I — CoppeliaSim + MATLAB Simulation

## I.1 Overview

This simulation uses **MATLAB** as the control engine and **CoppeliaSim** (formerly V-REP) as the physics simulator. MATLAB computes joint angles via inverse kinematics and gait planning algorithms, then sends real-time commands to CoppeliaSim through the **Legacy Remote API** (TCP, port `19999`).

Key features:
- Analytical forward and inverse kinematics for 3-DOF leg configurations
- Gait generation with cubic trajectory planning
- Navigation module with SLAM (lidarSLAM) and Augmented Kalman Filter (AKF)
- Synchronous simulation loop for deterministic execution

## I.2 Prerequisites

| Software | Version | Purpose |
| :--- | :--- | :--- |
| MATLAB | R2023a or later | Control logic, kinematics, trajectory planning |
| CoppeliaSim Edu | 4.6.0 (recommended) | Physics simulation engine |
| Robotics Toolbox for MATLAB | Latest | Robot model visualization (Peter Corke) |
| C Compiler (MinGW / GCC) | Latest | Required to load the Remote API library in MATLAB |

## I.3 Installing CoppeliaSim

### Windows

1. Download the **CoppeliaSim Edu** installer from the official website:
   ```
   https://www.coppeliarobotics.com/downloads
   ```
2. Select the appropriate version: **CoppeliaSim Edu, V4.6.0 rev18, Windows 64-bit**.
3. Run the `.exe` installer and follow the on-screen instructions.
4. Choose an installation directory (e.g., `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu`).
5. After installation, verify by launching `coppeliaSim.exe` from the installation folder.
6. Ensure the Remote API server is enabled:
   - Open CoppeliaSim.
   - Go to **Tools > User settings** and confirm that the **Remote API** is set to listen on port `19999`.
   - Alternatively, a child script in the simulation scene can start the server automatically.

### Linux (Ubuntu 22.04)

1. Download the Linux tarball from:
   ```
   https://www.coppeliarobotics.com/downloads
   ```
2. Select: **CoppeliaSim Edu, V4.6.0 rev18, Ubuntu 22.04, 64-bit**.
3. Extract the archive:
   ```bash
   cd ~/Downloads
   tar -xf CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04.tar.xz
   mv CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04 ~/CoppeliaSim
   ```
4. Install required system libraries:
   ```bash
   sudo apt update
   sudo apt install -y libgl1-mesa-dev libxkbcommon-x11-0 libxcb-xinerama0
   ```
5. Launch CoppeliaSim:
   ```bash
   cd ~/CoppeliaSim
   ./coppeliaSim.sh
   ```
6. If you encounter a `libQt5` error, install the Qt5 dependencies:
   ```bash
   sudo apt install -y qt5-default libqt5opengl5-dev
   ```

## I.4 Installing MATLAB

1. Download MATLAB from the MathWorks website:
   ```
   https://www.mathworks.com/products/matlab.html
   ```
2. Install MATLAB R2023a or later with the following toolboxes:
   - **Robotics System Toolbox** (or install Peter Corke's Robotics Toolbox via MATLAB Add-On Explorer)
   - **Navigation Toolbox** (required for `lidarSLAM`)
3. Install a C compiler for MEX files:
   - **Windows:** Install MinGW-w64 via MATLAB Add-On Explorer (search "MATLAB Support for MinGW-w64 C/C++ Compiler").
   - **Linux:** GCC is typically pre-installed. Verify with:
     ```bash
     gcc --version
     ```
4. Configure MATLAB to use the compiler:
   ```matlab
   mex -setup
   ```

## I.5 Clone the Repository

```bash
git clone https://github.com/longvotheengineer/quadruped-robot.git
cd quadruped-robot
git checkout develop
```

> **Note:** The `develop` branch is the default and contains the latest stable code. The `main` branch is reserved for releases.

## I.6 Project Structure (MATLAB)

All MATLAB simulation files are located under `Simulation/MATLAB/MotionControl/`:

```
Simulation/
├── CoppeliaSim/
│   ├── RobotDog3.ttt              # Main robot scene file
│   └── Navigation.ttt             # Navigation/SLAM scene file
└── MATLAB/
    └── MotionControl/
        ├── main.m                  # Entry point — connects to CoppeliaSim and runs the control loop
        ├── setup.m                 # Adds all subfolders to the MATLAB path
        ├── MotionControl/
        │   ├── ForwardKinematics.m # FK solver
        │   ├── InverseKinematics.m # IK solver (analytical, geometric decomposition)
        │   ├── Gait.m              # Gait trajectory waypoint generator
        │   ├── controlGait.m       # Gait execution controller
        │   ├── controlPath.m       # Path-level control orchestrator
        │   └── cubicPlanning.m     # Cubic polynomial trajectory planning
        ├── RemoteAPI/
        │   ├── remApi.m            # MATLAB wrapper for the CoppeliaSim Remote API
        │   ├── remoteApiProto.m    # Function signature prototypes
        │   └── remoteApi.dll       # Windows DLL (replace with .so for Linux)
        ├── Navigation/
        │   ├── AKF.m               # Augmented Kalman Filter
        │   ├── AStar.m             # A* path planning
        │   └── ...                 # SLAM and sensor processing utilities
        └── UnitSimulation/
            ├── test.m              # Quick kinematics test
            ├── testControlGait.m   # Gait controller test
            ├── testTrajectory.m    # Trajectory planning test
            └── ...                 # Additional unit tests and visualization scripts
```

## I.7 Setup and Configuration

### Step 1 — Verify the Remote API Library

The Remote API library must match your operating system:

| OS | Library File | Location |
| :--- | :--- | :--- |
| Windows (64-bit) | `remoteApi.dll` | `Simulation/MATLAB/MotionControl/RemoteAPI/` |
| Linux (64-bit) | `remoteApi.so` | Must be copied from CoppeliaSim installation |

**For Linux users:** Copy the shared library from your CoppeliaSim installation:

```bash
cp ~/CoppeliaSim/programming/remoteApiBindings/lib/lib/Ubuntu22_04/remoteApi.so \
   Simulation/MATLAB/MotionControl/RemoteAPI/
```

> **Important:** The library architecture (64-bit) must match your MATLAB architecture. Run `computer('arch')` in MATLAB to verify.

### Step 2 — Open the Scene in CoppeliaSim

1. Launch CoppeliaSim.
2. Open the scene file: **File > Open Scene** and navigate to:
   ```
   Simulation/CoppeliaSim/RobotDog3.ttt
   ```
3. Verify the scene contains the quadruped robot model with 12 revolute joints.
4. Ensure the Remote API child script is attached to an object in the scene.

### Step 3 — Configure MATLAB Path

1. Open MATLAB.
2. Navigate to `Simulation/MATLAB/MotionControl/`.
3. Run the setup script to add all subfolders to the path:
   ```matlab
   setup()
   ```

## I.8 Running the Simulation

1. **Start the simulation in CoppeliaSim** by clicking the **Play** button (or pressing `Ctrl+Shift+P`). The Remote API server will start listening on port `19999`.
2. **Run the main script in MATLAB:**
   ```matlab
   main
   ```
3. MATLAB will attempt to connect to CoppeliaSim at `127.0.0.1:19999`. If successful, you will see:
   ```
   Connected to remote API server
   ```
4. The control loop will begin executing, sending joint commands to the simulation.

### Running Unit Tests (Optional)

To test individual modules without CoppeliaSim:

```matlab
cd UnitSimulation

% Test kinematics
test

% Test gait controller
testControlGait

% Test trajectory planning
testTrajectory
```

## I.9 Expected Behavior

- The robot in CoppeliaSim should begin moving its legs according to the gait pattern.
- The MATLAB console will display `Connected to remote API server` followed by continuous control loop output.
- If the Navigation scene (`Navigation.ttt`) is used, a MATLAB figure window will show the SLAM map with the robot position overlay.

## I.10 Common Errors and Solutions

### Error: "Failed connecting to remote API server"

**Cause:** CoppeliaSim is not running or the Remote API server is not started.

**Solution:**
1. Ensure CoppeliaSim is open and the scene is loaded.
2. Press the **Play** button in CoppeliaSim before running `main.m`.
3. Verify the port by checking the CoppeliaSim status bar for `Remote API server started on port 19999`.

---

### Error: "Invalid MEX-file ... remoteApi"

**Cause:** No compatible C compiler is configured, or the library architecture does not match.

**Solution:**
1. Install a C compiler (MinGW on Windows, GCC on Linux).
2. Run `mex -setup` in MATLAB and select the installed compiler.
3. Verify the library matches MATLAB's architecture: `computer('arch')` should return `win64` or `glnxa64`.

---

### Error: "Undefined function or variable 'remApi'"

**Cause:** The MATLAB path does not include the `RemoteAPI` folder.

**Solution:**
Run the setup script:
```matlab
setup()
```
Or manually add the path:
```matlab
addpath('RemoteAPI')
```

---

### Error: "remoteApi.dll is not a valid Win32 application"

**Cause:** 32-bit DLL loaded in 64-bit MATLAB (or vice versa).

**Solution:**
Download the correct architecture version of `remoteApi.dll` from the CoppeliaSim installation:
```
<CoppeliaSim>/programming/remoteApiBindings/lib/lib/
```

---

### Error: "'lidarSLAM' requires Navigation Toolbox"

**Cause:** The Navigation Toolbox is not installed.

**Solution:**
Install via MATLAB Add-On Explorer or the command:
```matlab
matlab.addons.install('Navigation Toolbox')
```

---

### CoppeliaSim Scene Does Not Respond to MATLAB Commands

**Cause:** The simulation is not in synchronous mode, or the child script is missing.

**Solution:**
1. Verify the scene contains a child script that starts the Remote API server.
2. Ensure `sim.simxSynchronous(clientID, true)` is called in `main.m` (it is by default).
3. Restart both CoppeliaSim and MATLAB, then run again.

---

# Part II — Gazebo + ROS 2 Simulation

## II.1 Overview

This simulation uses **ROS 2 Humble** with **Gazebo Classic 11** for physics simulation. The robot is defined via a URDF model and controlled through the `ros2_control` framework with effort-based joint controllers. A Python-based gait generator (`leg_controller` package) computes joint trajectories and publishes commands to the Gazebo virtual motors.

Key features:
- URDF model with 12 revolute joints (4 legs, 3 DOF each)
- `ros2_control` integration via `gazebo_ros2_control` plugin
- `JointGroupEffortController` for torque-level control
- `JointStateBroadcaster` for encoder feedback
- RViz2 visualization with custom configuration
- Custom Gazebo world with tuned physics for legged locomotion
- Gait generation with quintic polynomial trajectory planning
- Command interface via ROS 2 topic (`/gait_control`)

## II.2 Prerequisites

| Software | Version | Purpose |
| :--- | :--- | :--- |
| Ubuntu | 22.04 LTS (Jammy Jellyfish) | Operating system |
| ROS 2 | Humble Hawksbill | Middleware framework |
| Gazebo Classic | 11 | Physics simulation |
| Python | 3.10 | Node development |
| colcon | Latest | Build tool |

> **Important:** This simulation is designed for **Linux only** (Ubuntu 22.04). WSL2 on Windows is not officially supported due to GPU passthrough requirements for Gazebo rendering.

## II.3 Installing ROS 2 Humble

Follow the official ROS 2 Humble installation guide for Ubuntu 22.04. The complete steps are reproduced below.

### Step 1 — Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Step 2 — Add the ROS 2 APT Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3 — Install ROS 2 Humble Desktop

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

### Step 4 — Source the ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add it to your shell configuration:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5 — Install Development Tools

```bash
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
```

### Step 6 — Verify ROS 2 Installation

Open two terminals:

**Terminal 1 (Talker):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Listener):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the listener printing "Hello World" messages from the talker.

## II.4 Installing Gazebo Classic 11

Gazebo Classic 11 is typically installed with `ros-humble-desktop`. Verify the installation:

```bash
gazebo --version
```

Expected output: `Gazebo multi-robot simulator, version 11.x.x`

If not installed, install manually:

```bash
sudo apt install gazebo -y
sudo apt install libgazebo-dev -y
```

## II.5 Installing Additional ROS 2 Dependencies

The following ROS 2 packages are required by this project:

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

Verify the key packages:

```bash
ros2 pkg list | grep gazebo_ros2_control
ros2 pkg list | grep ros2_controllers
```

Both commands should return the package names. If they do not, re-run the installation step above.

## II.6 Clone the Repository

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/longvotheengineer/quadruped-robot.git
cd quadruped-robot
git checkout develop
```

> **Note:** The `develop` branch contains the latest simulation code. The `main` branch is reserved for stable releases.

## II.7 Project Structure (ROS 2)

The ROS 2 workspace is located at `Software/Middleware/robot_ros2/`:

```
Software/Middleware/robot_ros2/
├── src/
│   ├── launch_main/                    # Top-level launch package
│   │   ├── launch/
│   │   │   └── launch_main.py          # Main launch file (starts all nodes)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── simulation/                     # Simulation description package
│   │   ├── urdf/
│   │   │   └── quadrupedRobot.urdf     # Robot URDF model (12 joints, ros2_control)
│   │   ├── launch/
│   │   │   └── launch_simulation.py    # Simulation launch (Gazebo, RViz, controllers)
│   │   ├── config/
│   │   │   └── ros2_gazebo_controller.yaml  # ros2_control controller configuration
│   │   ├── worlds/
│   │   │   └── custom_physics.world    # Gazebo world with tuned friction and physics
│   │   ├── rviz/
│   │   │   └── config.rviz             # RViz visualization configuration
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── leg_controller/                 # Gait control package
│   │   ├── leg_controller/
│   │   │   ├── nodeLegController.py    # Main ROS 2 node (entry point)
│   │   │   ├── gaitGenerator.py        # Gait pattern generator
│   │   │   ├── kinematics.py           # Inverse kinematics solver
│   │   │   ├── quinticPlanning.py      # Quintic polynomial trajectory planning
│   │   │   ├── controllerSim.py        # Gazebo joint state subscriber and publisher
│   │   │   └── serialPublish.py        # Joint command publisher interface
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── serial_driver/                  # Serial communication driver (for hardware)
│       └── ...
│
├── build/                              # colcon build output (auto-generated)
├── install/                            # colcon install output (auto-generated)
└── log/                                # colcon log output (auto-generated)
```

## II.8 Building the Workspace

### Step 1 — Navigate to the ROS 2 Workspace

```bash
cd ~/catkin_ws/src/quadruped-robot/Software/Middleware/robot_ros2
```

### Step 2 — Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### Step 3 — Build with colcon

```bash
colcon build --symlink-install
```

> **Note:** The `--symlink-install` flag creates symbolic links instead of copies, allowing you to edit Python files without rebuilding.

### Step 4 — Source the Workspace Overlay

```bash
source install/setup.bash
```

> **Important:** You must source the workspace overlay in **every new terminal** that needs to use these packages. Consider adding this to your shell configuration:
> ```bash
> echo "source ~/catkin_ws/src/quadruped-robot/Software/Middleware/robot_ros2/install/setup.bash" >> ~/.bashrc
> ```

## II.9 Running the Simulation

### Option A — Full Launch (Recommended)

This starts all nodes at once: Gazebo, RViz, robot model, controllers, and the gait controller node.

**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch launch_main launch_main.py
```

### Option B — Step-by-Step Launch

If you need more control or want to debug individual components:

**Terminal 1 — Launch Gazebo and Controllers:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simulation launch_simulation.py
```

Wait until you see the Gazebo window with the robot model spawned. Then verify the controllers:

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster [joint_state_broadcaster/JointStateBroadcaster] active
leg_controller          [effort_controllers/JointGroupEffortController] active
```

**Terminal 2 — Launch the Gait Controller Node:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run leg_controller node_leg_controller
```

You should see:
```
[INFO] [node_leg_controller]: The main Node has started.
```

Once the node receives encoder data from Gazebo, it will output:
```
[INFO] [node_leg_controller]: Init pose reached. Waiting for commands.
```

## II.10 Publishing Gait Commands

Once the system is running and the node reports `Waiting for commands`, you can send gait commands via the `/gait_control` topic.

The command format is:

```
<COMMAND> <STEP_COUNT>
```

- `COMMAND` — One of the predefined gait or body-motion identifiers listed below.
- `STEP_COUNT` — Number of complete gait cycles to execute. Set to `0` for continuous (infinite) execution.

### Example Commands

**Terminal 3:**

```bash
# Trot forward continuously
ros2 topic pub --once /gait_control std_msgs/msg/String "{data: 'TROT_FORWARD 0'}"

# Walk forward for 5 gait cycles, then hold position
ros2 topic pub --once /gait_control std_msgs/msg/String "{data: 'WALK_FORWARD 5'}"

# Execute body sway oscillation continuously
ros2 topic pub --once /gait_control std_msgs/msg/String "{data: 'BODY_SWAY 0'}"

# Return to the zero (homing) position
ros2 topic pub --once /gait_control std_msgs/msg/String "{data: 'ZERO 0'}"
```

### Command Reference

The table below lists all supported gait commands. Each command combines a **direction** with a **gait pattern** (phase shift configuration) and a **trajectory type**.

#### Locomotion Commands

| Command | Gait Pattern | Trajectory | Description |
| :--- | :--- | :--- | :--- |
| `TROT_FORWARD` | Trot (diagonal pairs) | D-shape foot path | Move forward in trot gait — diagonal legs swing simultaneously |
| `TROT_BACKWARD` | Trot (diagonal pairs) | D-shape foot path (reversed) | Move backward in trot gait |
| `WALK_FORWARD` | Walk (sequential) | D-shape foot path | Move forward in walk gait — legs swing one at a time in sequence |
| `WALK_BACKWARD` | Walk (sequential) | D-shape foot path (reversed) | Move backward in walk gait |
| `TURN_RIGHT` | Trot (diagonal pairs) | Left legs forward, right legs reversed | Rotate the body clockwise (turn right) |
| `TURN_LEFT` | Trot (diagonal pairs) | Right legs forward, left legs reversed | Rotate the body counter-clockwise (turn left) |

#### Body Motion Commands

| Command | Phase Pattern | Trajectory | Description |
| :--- | :--- | :--- | :--- |
| `BODY_PUSHUP` | All legs in-phase | Vertical oscillation | All four legs extend and retract simultaneously — push-up motion |
| `BODY_SWAY` | Left/right pairs offset by 50% | Vertical oscillation | Body sways side-to-side — left and right legs oscillate in anti-phase |
| `BODY_CIRCLE` | Sequential 25% offset per leg | Vertical oscillation | Body traces a circular path — each leg is phase-shifted by 90 degrees |

#### System Commands

| Command | Description |
| :--- | :--- |
| `ZERO` | Move all joints to the homing (zero) position via linear interpolation from current pose |

### Monitoring Joint States

To inspect the current joint positions in real-time:

```bash
ros2 topic echo /joint_states
```

### Monitoring Controller Effort Commands

To inspect the effort (torque) commands being sent to the virtual motors:

```bash
ros2 topic echo /leg_controller/commands
```

## II.11 Expected Behavior

1. **Gazebo Window:** The quadruped robot appears in the custom world. The body is dark grey, hip links are red, upper legs are green, and lower legs are blue.
2. **RViz Window:** The same robot model is visualized with TF frames displayed.
3. **Initial State:** After the `node_leg_controller` starts, the robot holds its initial standing pose.
4. **After Gait Command:** The robot begins executing the requested gait pattern. Legs move in coordinated pairs (trot gait), with trajectory following quintic polynomial profiles.

## II.12 Common Errors and Solutions

### Error: "package 'gazebo_ros2_control' not found"

**Cause:** The `gazebo_ros2_control` package is not installed.

**Solution:**
```bash
sudo apt install ros-humble-gazebo-ros2-control -y
```

---

### Error: "Could not find controller_manager"

**Cause:** The `ros2_control` packages are not installed.

**Solution:**
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers -y
```

---

### Error: "`colcon build` fails with 'setup.py install is deprecated'"

**Cause:** Newer versions of `setuptools` have deprecated the `setup.py install` method.

**Solution:**
```bash
pip install setuptools==58.2.0
```

Then rebuild:
```bash
colcon build --symlink-install
```

---

### Error: "No executable found" when running `ros2 launch`

**Cause:** The workspace overlay has not been sourced.

**Solution:**
```bash
source install/setup.bash
```

---

### Error: "Gazebo process died" or Gazebo Crashes on Launch

**Cause:** GPU driver issues or insufficient system resources.

**Solution:**
1. Update your GPU drivers:
   ```bash
   sudo ubuntu-drivers autoinstall
   sudo reboot
   ```
2. If running in a virtual machine, try software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ros2 launch launch_main launch_main.py
   ```
3. Check system resources (minimum 4 GB RAM recommended):
   ```bash
   free -h
   ```

---

### Error: "Could not load controller 'leg_controller'" or Controllers Remain Inactive

**Cause:** The controller YAML configuration does not match the URDF `ros2_control` definition, or the `gazebo_ros2_control` plugin is not loaded.

**Solution:**
1. Verify the URDF contains the `<gazebo>` plugin block for `libgazebo_ros2_control.so`.
2. Verify the YAML file (`ros2_gazebo_controller.yaml`) lists all 12 joint names correctly.
3. Ensure the joint names in the YAML file exactly match those in the URDF (case-sensitive).
4. Re-build and re-source:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

### Error: "Robot spawns and immediately falls through the ground"

**Cause:** The URDF is missing collision or inertial properties, or the spawn height is incorrect.

**Solution:**
1. All links in the URDF must have `<collision>` and `<inertial>` elements.
2. Check the spawn position in `launch_simulation.py`. The robot is spawned at `z=0.4` by default to account for leg length.
3. Verify the custom world file (`custom_physics.world`) is loaded correctly.

---

### Error: "'joint_state_broadcaster' spawner times out"

**Cause:** The controller manager has not fully initialized when the spawner is invoked.

**Solution:**
Add a delay before the spawner launches, or re-launch the system. The timing issue is intermittent and typically resolves on the second attempt:
```bash
ros2 launch launch_main launch_main.py
```

---

### Error: "Failed to load plugin libgazebo_ros2_control.so"

**Cause:** The `gazebo_ros2_control` library is not in the Gazebo plugin path.

**Solution:**
1. Set the plugin path explicitly:
   ```bash
   export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
   ```
2. Add this to your `~/.bashrc` for persistence:
   ```bash
   echo "export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### Warning: "URDF does not contain transmission elements"

**Cause:** A mismatch between the URDF and the `ros2_control` hardware interface.

**Solution:** This warning can be safely ignored when using `gazebo_ros2_control` with the `<ros2_control>` tag in the URDF (as this project does). The `GazeboSystem` plugin handles the hardware interface directly.

---

## Contributing

Contributions are welcome. Please follow these guidelines:

1. Fork the repository and create a feature branch from `develop`.
2. Follow the existing code style and naming conventions.
3. Test your changes with both simulation environments before submitting a pull request.
4. Open an issue first to discuss significant changes.

## Contact

**Author:** LvDaengineer  
**Email:** linhlong875@gmail.com  
**GitHub:** [longvotheengineer](https://github.com/longvotheengineer)

---

<p align="center">
  <em>Designed and developed by LvDaengineer</em>
</p>
