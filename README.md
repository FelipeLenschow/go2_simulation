# Unitree Go2 Simulation and Real Robot Execution

This guide provides instructions on how to run the Unitree Go2 robot in both the Gazebo simulation environment and on the physical hardware.

## Prerequisites
Ensure that your ROS 2 installation is sourced, and that you have built the workspace. Navigate to the root of your ROS 2 workspace (where this package is located) and run:
```bash
# Source your ROS 2 distribution (Jazzy in this case)
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
colcon build

# Source the local workspace overlay
source install/setup.bash
```

---

## Running in Gazebo Simulation

To run the simulation in Gazebo, which spawns the Go2 robot, sets up the joint controllers, and opens RViz:

```bash
ros2 launch go2_gzsim go2.launch.py
```

Optional arguments:
- `simulation:=true/false` (default: `true`)
- `fixed_base:=true/false` (default: `false`): Fixes the robot base in the air.
- `use_sim_time:=true/false` (default: `true`)

---

## Testing Movements in Simulation

Once the Gazebo simulation is running, it expects low-level joint commands. You can run the included `go2_refs` example node, which sends a pre-programmed sequence of joint angles to make the robot move.

Open a **new terminal**, navigate to the workspace, source the environment, and run the node:

```bash
cd ~/go2_simulation
source install/setup.bash
ros2 run go2_refs go2_refs
```

The robot should begin cycling through a series of leg extension movements in Gazebo and RViz.

---

## Running on the Real Robot

Running the real robot is divided into two parts: running the hardware drivers on the robot's onboard computer and running the visualization/operator tools on a base station (operator PC).

### 1. Onboard the Robot
SSH into the Go2's onboard computer, source the workspace, and run the robot launch file. This starts the robot state publisher, ros2_control manager, joint controllers, and remapping nodes.

```bash
ros2 launch go2_real go2.robot.launch.py
```

### 2. On the Operator PC
On your local machine (operator PC) connected to the same ROS 2 network, run the operator launch file to visualize the robot's state in RViz and start the necessary operator remapping nodes.

```bash
ros2 launch go2_real go2.operator.launch.py
```
