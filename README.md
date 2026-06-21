# Ball Tracking System

An engineering-oriented ROS2 + Orange Pi + ESP32 tracked robot project for ball detection, tracking, gripping, and safe-zone placement.

## Overview

This repository contains a complete small-robot workflow:

- YOLO/RKNN-based ball and safe-zone detection on Orange Pi
- ROS2 nodes for perception, motion logic, and serial communication
- ESP32 motor-control firmware with encoder closed-loop control
- Mission logic for:
  - startup push-out
  - ball search
  - approach and grab
  - safe-zone search
  - placement and retreat

## Repository Structure

```text
ball-tracking-system/
├─ src/
│  ├─ ball_detection/
│  │  ├─ ball_detection/
│  │  └─ launch/
│  └─ ball_interfaces/
└─ arduino/
   └─ gongchuang_mar31a/
```

## Main Components

### ROS2 packages

- `src/ball_detection`
  - `detect_node.py`: RKNN/YOLO-based object detection
  - `ball_mission_logic.py`: full mission state machine
  - `esp32_serial_node.py`: `/cmd_vel` to ESP32 serial bridge
  - `tracked_launcher.launch.py`: integrated launch entry
- `src/ball_interfaces`
  - custom ROS2 messages and services used by the system

### Arduino / ESP32

- `arduino/gongchuang_mar31a/gongchuang_mar31a.ino`
  - dual-motor PID control
  - encoder feedback
  - servo control
  - serial command parsing

## Hardware / Software Stack

- Orange Pi
- ROS2 Humble
- RKNNLite
- USB camera
- ESP32
- tracked chassis with encoder motors

## Quick Start

### 1. Build ROS2 workspace

Place the `src` folder into a ROS2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select ball_interfaces ball_detection
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Launch the integrated system

```bash
ros2 launch ball_detection tracked_launcher.launch.py \
  start_detect:=true \
  start_mission_logic:=true \
  start_serial_bridge:=true \
  team_color:=red
```

### 3. Flash the ESP32 firmware

Open:

```text
arduino/gongchuang_mar31a/gongchuang_mar31a.ino
```

and upload it with Arduino IDE after selecting the correct ESP32 board and serial port.

## Notes

- The repository includes RKNN/ONNX model files used during deployment and debugging.
- Camera parameters, wheel diameter, encoder pulses per wheel revolution, and serial port may need calibration on the target robot.
- The current implementation was tuned for a real tracked robot platform rather than a generic simulator.

## Status

This is an actively iterated engineering project and reflects real deployment-oriented development, including perception, control, and hardware integration.
