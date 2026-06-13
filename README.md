# 3-DOF Serial Robot Manipulator with ROS2 & MoveIt2

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg) ![Docker](https://img.shields.io/badge/Docker-Containerized-2496ED.svg) ![MoveIt2](https://img.shields.io/badge/MoveIt2-Path%20Planning-orange.svg) ![Hardware](https://img.shields.io/badge/Hardware-ESP32-brightgreen.svg)

## Overview
This repository contains the complete software and hardware architecture for a **3 Degrees of Freedom (DOF) Serial Robot Manipulator**. Designed from scratch, the system integrates advanced kinematics, real-time hardware control, and high-level trajectory planning using **ROS2** and **MoveIt2**.

The project spans multiple disciplines, including 3D mechanical design (Autodesk Inventor), embedded systems (ESP32/Arduino), and distributed robotic operating systems.

## Key Features

*   **Docker Containerization**: The entire ROS2 Jazzy ecosystem is containerized via Docker. This eliminates dependency conflicts, ensuring cross-platform reproducibility and a clean "plug-and-play" development environment.
*   **Custom 3D Mechanical Design**: A robust articulated arm design modeled in Autodesk Inventor (`.ipt` and `.iam` files). Components were manufactured using 3D printing, optimized for torque and structural rigidity.
*   **ROS2 Architecture**: Fully containerized in a ROS2 workspace with dedicated packages for:
    *   `mechanism_308_description`: URDF models for accurate 3D visualization in RViz.
    *   `mechanism_308_moveit_config`: MoveIt2 configuration for collision-aware inverse kinematics (IK) and trajectory planning.
    *   `mechanism_308_hardware`: Custom `ros2_control` hardware interface bridging the digital twin with physical ESP32 controllers.
*   **Denavit-Hartenberg (DH) & Kinematics**: Forward and inverse kinematics were parametrically solved using the DH convention. Jacobian analysis was performed to identify and avoid singular conditions during path planning.
*   **Free-Drive & Teaching Mode**: Features a custom Python node (`free_drive.py`) that disables motor holding torque, allowing the user to physically guide the arm. The joint states are reflected in RViz in real-time, enabling "teach-and-repeat" functionality and dynamic home calibration.
*   **Real-Time Hardware-in-the-Loop (HIL)**: Serial communication routing between Windows (WSL) and the ESP32 microcontrollers ensures low-latency execution of MoveIt2 generated joint space trajectories.

## Project Structure

*   `CAD-Models/`: Exported `mechanism_308.step` 3D model for universal CAD compatibility, synced with ROS2 naming.
*   `ROS2-Workspace/`: The main `colcon` workspace containing the ROS2 packages and Python execution nodes.
*   `photos/`: Visual documentation of the physical robot and UI interfaces.
*   `Motor_Test/` & `Motor_PID_Test/`: Embedded C/C++ firmware for the ESP32 to handle PID velocity/position control and encoder reading.

## Getting Started
*(Please note: Running the full hardware stack requires WSL port routing and physical ESP32 connections.)*

1.  Navigate to the workspace: `cd ROS2-Workspace`
2.  Source your ROS2 installation: `source /opt/ros/jazzy/setup.bash`
3.  Build the packages: `colcon build`
4.  Launch the simulation/hardware: `ros2 launch mechanism_308_moveit_config demo.launch.py`
5.  To activate manual teaching: `ros2 run ros2_console_tools free_drive.py`

---
*This repository serves as a professional portfolio artifact demonstrating expertise in Mechatronics Engineering, Kinematics, and ROS2 ecosystem development.*
