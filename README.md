# AMR Trajectory Tools

A ROS1 Noetic package for visualizing and storing trajectory data of Autonomous Mobile Robots (AMRs). This package provides tools to collect, save, and visualize robot trajectories

## Overview

This package contains two ROS nodes:

1. **Trajectory Publisher and Saver Node**: Collects robot trajectory data, visualizes it in RViz, and provides a service to save the data to csv format
2. **Trajectory Reader and Publisher Node**: Reads saved trajectory files and publishes the data for visualization

## Features

- Real-time trajectory visualization in RViz
- Time-limited trajectory storage
- Frame transformation support
- Customizable marker appearance

## Installation

### Prerequisites

- ROS1 Noetic
- tmux 

### Setup (one time)

- sudo apt-get install ros-noetic-turtlesim -y
- sudo apt install tmux tmuxp -y
- echo alias tmuxp='export LANG=en_IN.utf8; tmuxp' >> ~/.bashrc 

### Building

```bash
# Create a workspace (if you don't have one already)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

place the amr_trajectory_tools directory

# Build
cd ~/catkin_ws
catkin build

# Run 
source devel/setup.bash
cd src/amr_trajectory_tools/
tmuxp load tmux.yaml

Note: Run either the tmux session or the commands mentioned below
```

## Usage

### Launch turtlesim

```bash
rosrun turtlesim turtlesim_node

# Node to navigate the turtlesim in random direction

rosrun amr_trajectory_tools publish_random_twist
```

### Trajectory Publisher and Saver Node

This node subscribes to the robot's pose topic, visualizes the trajectory, and provides a service to save the trajectory to a file

```bash
# Launch with default parameters
roslaunch amr_trajectory_tools trajectory_publisher_saver.launch

# Save trajectory using the service
rosservice call /save_trajectory "filename: 'trajectory_data.csv' duration: 100.0"
```

Duration parameter: Time in seconds

### Trajectory Reader and Publisher Node 

This node reads a saved trajectory file and publishes it for visualization in RViz

```bash
rosrun amr_trajectory_tools trajectory_reader_publisher _trajectory_file:=trajectory_data.csv
```
#### Note: 
- The trajectory_reader_publisher node must be executed regardless of whether the system is launched using tmux or run manually through individual commands
- The same CSV file name used in the service call to save the trajectory must also be provided to the trajectory_reader_publisher node for correct visualization

## Topics

The following ROS topics are used in the trajectory-related nodes:

- **`/turtle1/pose`** → Subscribed topic for receiving the robot's pose
- **`/saved_trajectory_markers`** → Published by the **Trajectory Publisher and Saver Node** for visualizing the saved trajectory (**red color**)
- **`/robot_trajectory_markers`** → Published by the **Trajectory Reader and Publisher Node** for replaying a saved trajectory in RViz (**green color**)
- **`/save_trajectory`** → Service to save the trajectory data to a file
