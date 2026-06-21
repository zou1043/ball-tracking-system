# Ball Tracking System / 小球跟踪抓取系统

An engineering-oriented ROS2 + Orange Pi + ESP32 tracked robot project for ball detection, tracking, gripping, and safe-zone placement.  
这是一个面向工程落地的 ROS2 + 香橙派 + ESP32 履带小车项目，实现了小球识别、跟踪、抓取以及安全区放置的完整流程。

## Overview / 项目简介

This repository contains a complete small-robot workflow:  
本仓库整理了一个完整的小车任务链路，包括：

- YOLO/RKNN-based ball and safe-zone detection on Orange Pi  
  基于 YOLO / RKNN 的小球与安全区视觉识别
- ROS2 nodes for perception, motion logic, and serial communication  
  基于 ROS2 的感知、运动逻辑与串口通信节点
- ESP32 motor-control firmware with encoder closed-loop control  
  带编码器闭环控制的 ESP32 电机控制固件
- Mission logic for startup push-out, ball search, approach, grabbing, safe-zone search, placement, and retreat  
  包含开局冲球、找球、接近、抓取、找安全区、放球、撤离等任务状态机

## Repository Structure / 仓库结构

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

## Main Components / 主要组成

### ROS2 packages / ROS2 功能包

- `src/ball_detection`
  - `detect_node.py`: RKNN / YOLO object detection  
    目标检测节点，负责小球和安全区识别
  - `ball_mission_logic.py`: full mission state machine  
    任务状态机，负责找球、抓球、送球、撤离等逻辑
  - `esp32_serial_node.py`: `/cmd_vel` to ESP32 serial bridge  
    串口桥接节点，将 ROS2 速度指令转换为 ESP32 下位机指令
  - `tracked_launcher.launch.py`: integrated launch entry  
    总启动入口
- `src/ball_interfaces`
  - custom ROS2 messages and services used by the system  
    系统使用的自定义消息与服务

### Arduino / ESP32

- `arduino/gongchuang_mar31a/gongchuang_mar31a.ino`
  - dual-motor PID control  
    双电机 PID 控制
  - encoder feedback  
    编码器反馈闭环
  - servo control  
    舵机夹爪控制
  - serial command parsing  
    串口命令解析

## Hardware / Software Stack / 硬件与软件环境

- Orange Pi / 香橙派
- ROS2 Humble
- RKNNLite
- USB camera / USB 摄像头
- ESP32
- tracked chassis with encoder motors / 带编码器电机的履带底盘

## Quick Start / 快速开始

### 1. Build ROS2 workspace / 编译 ROS2 工作区

Place the `src` folder into a ROS2 workspace:  
将 `src` 文件夹放入 ROS2 工作区后执行：

```bash
cd ~/ros2_ws
colcon build --packages-select ball_interfaces ball_detection
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Launch the integrated system / 启动整套系统

```bash
ros2 launch ball_detection tracked_launcher.launch.py \
  start_detect:=true \
  start_mission_logic:=true \
  start_serial_bridge:=true \
  team_color:=red
```

### 3. Flash the ESP32 firmware / 烧录 ESP32 固件

Open / 打开：

```text
arduino/gongchuang_mar31a/gongchuang_mar31a.ino
```

Then upload it with Arduino IDE after selecting the correct ESP32 board and serial port.  
在 Arduino IDE 中选择正确的 ESP32 开发板和串口后完成烧录。

## Notes / 说明

- The repository includes RKNN / ONNX model files used during deployment and debugging.  
  仓库中保留了部署和调试时使用的 RKNN / ONNX 模型文件。
- Camera parameters, wheel diameter, encoder pulses per wheel revolution, and serial port may need calibration on the target robot.  
  摄像头参数、轮径、编码器每圈脉冲数以及串口号，都需要按实机重新标定。
- The current implementation was tuned for a real tracked robot platform rather than a generic simulator.  
  当前实现是围绕真实履带小车调试得到的，而不是面向通用仿真环境。

## Status / 项目状态

This is an actively iterated engineering project and reflects real deployment-oriented development, including perception, control, and hardware integration.  
这是一个持续迭代中的工程型项目，强调真实部署中的感知、控制和软硬件联调能力。
