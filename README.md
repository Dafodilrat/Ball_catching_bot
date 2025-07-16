# 🏀 Ball Catching Bot — ROS 2 Powered Mecanum Robot

This repository contains the ROS 2 implementation for a mobile robot that tracks, localizes, and navigates to intercept a projectile (ball). The robot is based on the **rosmaster x3** platform, running on an NVIDIA Jetson Nano and fully containerized with Docker.

---

## 🧠 Architecture Overview

The system is organized into four major modules:

- **Trajectory Tracking** — Predicts ball landing point using `/rosmaster` node.
- **Control Module** — PID-based velocity controller for 4 mecanum wheels.
- **Sensor Inputs** — Integrates Astra camera, IMU, and encoder data.
- **Localization** — EKF fusing IMU and encoder data to provide stable `/odom`.

---

## 🧩 ROS 2 Packages

| Package | Description |
|--------|-------------|
| `pkg_motor_driver` | Converts `/cmd_vel` into low-level motor commands |
| `pkg_pid_controller` | PID-based control for tracking x, y, yaw trajectories |
| `rosmaster` | Publishes predicted projectile landing coordinates |
| `robot_description` | Contains robot URDF and state publisher |
| `astra_camera` | Depth and RGB camera interface |
| `robot_localization` | EKF-based fusion of IMU and encoder for accurate localization |

---

## 🦾 Robot Control

The robot uses 3 PID controllers for linear `x`, `y` and angular `yaw` control. Final wheel speeds are computed with mecanum kinematic equations:

```math
M1 = (Vy - Vx + (L+W) * Wz) / R
M2 = (Vy + Vx + (L+W) * Wz) / R
M3 = (Vy - Vx - (L+W) * Wz) / R
M4 = (Vy + Vx - (L+W) * Wz) / R
```

Each motor command is sent to the driver node via `/cmd_vel`.

---

## 🧭 Localization

Uses an Extended Kalman Filter (EKF) via `robot_localization` package:
- Inputs: IMU (acceleration, gyro) + Encoders
- Output: `/odom` transform and velocity estimates
- Config: `robot_localization/config/ekf.yaml`

---

## 🗂️ Launch Files

```bash
ros2 launch bringup.launch.py           # Launch all nodes
ros2 launch pkg_motor_driver/start_base.launch.py
ros2 launch pkg_pid_controller/controller.launch.py
ros2 launch rosmaster/master.launch.py
ros2 launch astra_camera/astra.launch.py
```

---

## 🐳 Docker Setup

### ✅ [Dockerfile](Dockerfile)

This project includes a full Docker environment for ROS 2 Humble and all dependencies. It is designed to run on Jetson Nano with Ubuntu 18.04 (via container).

To build:

```bash
docker build -t ball_bot .
```

To run:

```bash
docker compose up
```

See [`docker-compose.yml`](docker-compose.yml) for hardware passthrough and NVIDIA GPU access configuration.

---

## 🔍 Camera Calibration

Calibrated using ROS 2 camera calibration tools and OpenCV:
- Corner detection via `findChessboardCorners`
- Checkerboard images mapped to real-world distances
- Calibration data saved to `/camera/*/camera_info`

---

## 📸 Images (Suggestions)
Recommended visuals for this README:
- PID response graphs (desired vs actual x/y)
- ROS 2 RQT Graph screenshots
- Robot localization diagram (EKF fusion)
- Astra camera depth map snapshot
- URDF model visualization

---

## 🖥️ Hardware Platform

- Jetson Nano (Ubuntu 18.04 + Docker)
- 4x Mecanum Wheels with Encoders
- Astra Pro RGB-D Camera
- IMU (9-DOF)
- Motor Driver connected via USB/serial

---

## 🧰 Dependencies

Install system packages:

```bash
sudo apt update
sudo apt install libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev   libeigen3-dev udev build-essential cmake   ros-humble-diagnostic-updater ros-humble-geographic-msgs   nlohmann-json3-dev
```

Build workspace:

```bash
cd ros2_ws
colcon build
```

---

## 📄 License

This repository is part of an academic project and is intended for educational use only.