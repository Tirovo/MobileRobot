# MobileRobotROS2

**MobileRobotROS2** is a multi-processor robotics project combining real-time embedded systems, computer vision, and wireless communication through the ROS 2 middleware. This mobile robot is capable of operating in multiple autonomous and semi-autonomous modes, with an intuitive control interface hosted on a PC.

## 📌 Project Objectives

The goal of this project is to program a mobile robot and its control interface to support the following operating modes:

- **Manual Mode**: Remote control of the robot’s direction and speed with real-time obstacle detection.
- **Random Mode**: The robot moves freely, avoids obstacles, and changes direction periodically.
- **Tracking Mode**: The robot follows a colored target detected by its camera.

## 🧩 System Components

### 🔧 STM32 Nucleo-F411 Board
- Closed-loop motor control
- Distance sensor acquisition
- LCD display handling
- Communication with the Raspberry Pi
- Runs a Real-Time Operating System (RTOS)

### 🍓 Raspberry Pi
- Webcam image acquisition
- Image processing (for target tracking)
- Communication with the host PC via Wi-Fi

### 🖥️ Host PC with GUI
- Provides a user interface (UI) to:
  - Set the robot's operating mode
  - Send movement commands (speed, direction)
  - Display sensor values
  - Display the webcam feed

## 🔄 Communication Architecture

All inter-device communication is based on the **ROS 2** messaging framework, ensuring modularity and scalability:

- STM32 ⬌ Raspberry Pi (via ROS 2 messages over UART or serial bridge)
- Raspberry Pi ⬌ PC (via ROS 2 messages over Wi-Fi)

## 📸 Hardware Overview

To be included

## ⚙️ Technologies Used

To be included

## 📁 Folder Structure

MobileRobotROS2/
├── stm32_firmware/ # RTOS project for motor & sensor control
├── rpi_vision_node/ # ROS 2 node for image processing
├── pc_gui/ # PC interface to control the robot
├── launch/ # ROS 2 launch files
├── config/ # Parameter/config files
├── assets/ # Diagrams and system photos
└── README.md # Project documentation

## 📚 License

This project is licensed under the MIT License.
