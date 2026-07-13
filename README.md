# Roboracer Data Visualizer
Real-time data visualization tool for Roboracer autonomous vehicles, built with pyqtgraph and ROS2.

## Description
This node provides a graphical interface to monitor vehicle state during experiments. It displays real-time plots of position, speed, acceleration, ERPM, and trajectory tracking. Features Start/Pause/Reset buttons, automatic ROS bag recording, and experiment export to CSV, MATLAB, and images.

## Features
-Real-time plotting with 5 tabbed views
-Trajectory tracking (expected vs actual)
-Error computation (speed, position X/Y)
-Start/Pause/Reset control buttons
-Automatic ROS bag recording
-Data export: CSV, .mat, PNG, SVG
-Keyboard shortcut: Ctrl+S to save
## Requirements
-ROS2 Humble
-Python 3.8+
-PyQt5
-pyqtgraph
-scipy
-numpy
## Installation
```
sudo apt install python3-pyqt5 python3-pyqtgraph
pip install scipy numpy

cd ~/ros2_ws/src
git clone https://github.com/hugocraft586-source/roboracer-visualizer.git
cd ~/ros2_ws
colcon build --packages-select plotter vesc_msgs
source install/setup.bash
```
## Usage
ros2 run plotter plotter

## Buttons
- Start (Green): Begin data collection and ROS bag recording
- Pause (Orange): Stop collection and ROS bag, keep data in memory
- Reset (Red): Clear all data, reset timestamps, disable save
- Save Data (Blue): Export all data (CSV, .mat, PNG, SVG) - also Ctrl+S

## Workflow
- Enter experiment name in the text field (optional)
- Press Start to begin recording
- Press Pause to stop recording temporarily
- Press Save Data or Ctrl+S to export
- Press Reset to clear and start a new experiment

## Topics
### Subscribers
- ``/odometry/filtered`` (nav_msgs/Odometry): Filtered vehicle odometry (position, speed, yaw)
- ``/commands/motor/speed`` (std_msgs/Float64): Motor speed command in ERPM
- ``/sensors/imu/raw`` (sensor_msgs/Imu): IMU linear acceleration (X, Y)
- ``/sensors/core`` (vesc_msgs/VescStateStamped): VESC core state (ERPM feedback)
- ``/sequencer/expected/x`` (std_msgs/Float64): Expected X position from sequencer
- ``/sequencer/expected/y`` (std_msgs/Float64): Expected Y position from sequencer
- ``/sequencer/trajectory`` (std_msgs/Float64MultiArray): Expected trajectory waypoints

## Tabs Overview
- Position X: Odometry X vs Expected X + Position X error
- Position Y: Odometry Y vs Expected Y + Position Y error
- Speed: Reference speed vs Real speed + Speed error
- Acceleration / ERPM: IMU linear acceleration (X/Y) + VESC ERPM vs Input ERPM
- Trajectory: 2D XY plot: Odometry vs Expected trajectory
