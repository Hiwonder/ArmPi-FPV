# ArmPi FPV

English | [中文](https://github.com/Hiwonder/ArmPi-FPV/blob/main/README_cn.md)

<p align="center">
  <img src="./sources/images/image.webp" alt="ArmPi FPV Logo" width="400"/>
</p>

## Product Overview

ArmPi FPV is an intelligent vision robotic arm controlled by Raspberry Pi 5/Raspberry Pi 4B, using OpenCV as the image processing library. Equipped with a high-definition 170° wide-angle camera, it has first-person vision capabilities. It adopts the ROS (Robot Operating System) and has built-in MoveIt kinematics. Through Python programming, it can implement multiple AI vision recognition functions such as face recognition, color tracking, palletizing, and smart warehousing! It not only satisfies users' learning and verification needs for machine vision, robotic arm motion control, and torque control algorithms, but also provides a fast and convenient integration solution for secondary development of hand-eye coordination and visual grasping.

## Official Resources

### Official Hiwonder
- **Official Website**: [https://www.hiwonder.net/](https://www.hiwonder.net/)
- **Product Page**: [https://www.hiwonder.com/products/armpi-fpv](https://www.hiwonder.com/products/armpi-fpv)
- **Official Documentation**: [https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/)
- **Technical Support**: support@hiwonder.com

## Key Features

### AI Vision Functions
- **Face Detection** - Comprehensive face recognition capabilities
- **Color Tracking** - Real-time color-based object tracking
- **Object Tracking** - Advanced object detection and tracking
- **Visual Grasping** - Hand-eye coordination for precise object manipulation
- **AprilTag Recognition** - Precision tag-based positioning

### Advanced Applications
- **Object Palletizing** - Automated stacking and palletizing operations
- **Smart Warehousing** - Intelligent warehouse management and sorting
- **Object Sorting** - Color and shape-based sorting automation
- **Multi-Robot Control** - Coordinated control of multiple robotic arms
- **Voice Control** - ASR-powered voice commands

### Motion Control
- **Inverse Kinematics** - Advanced motion planning algorithms
- **MoveIt Integration** - Full MoveIt motion planning support
- **Torque Control** - Precise force and torque management
- **Trajectory Planning** - Smooth path generation and execution
- **Servo Control** - High-precision bus servo management

### Programming Interface
- **ROS Integration** - Complete Robot Operating System support
- **Python Programming** - Comprehensive Python SDK
- **MoveIt API** - Motion planning and control interface
- **OpenCV Library** - Computer vision and image processing
- **Open Source** - Complete open-source platform for customization

## Hardware Configuration
- **Processor**: Raspberry Pi 5 or Raspberry Pi 4B
- **Operating System**: ROS Noetic compatible Linux
- **Vision System**: HD 170° wide-angle camera
- **Servos**: High-precision bus servos
- **DOF**: Multi-degree-of-freedom robotic arm
- **Communication**: WiFi, Bluetooth

## Project Structure

```
armpi_fpv/
├── src/                          # ROS source packages
│   ├── armpi_fpv_bringup/        # System startup and configuration
│   ├── armpi_fpv_common/         # Common utilities and libraries
│   ├── armpi_fpv_descrption/     # Robot URDF descriptions
│   ├── armpi_fpv_kinematics/     # Kinematics algorithms
│   ├── armpi_fpv_moveit_config/  # MoveIt configuration
│   ├── asr_control/              # Voice control module
│   ├── face_detect/              # Face detection
│   ├── hiwonder_servo_controllers/ # Servo controllers
│   ├── hiwonder_servo_driver/    # Servo hardware drivers
│   ├── hiwonder_servo_msgs/      # Servo message definitions
│   ├── lab_config/               # Color calibration configuration
│   ├── multi_control/            # Multi-robot control
│   ├── object_pallezting/        # Palletizing application
│   ├── object_sorting/           # Object sorting application
│   ├── object_tracking/          # Object tracking application
│   ├── ros_robot_controller/     # Hardware controller interface
│   └── warehouse/                # Smart warehouse application
├── build/                        # Build files
├── devel/                        # Development files
└── sources/                      # Resources and documentation
```

## Version Information
- **Current Version**: ArmPi FPV v1.0.0
- **Supported Platform**: Raspberry Pi 5, Raspberry Pi 4B
- **ROS Version**: ROS1 (Noetic)

### Related Technologies
- [ROS](http://www.ros.org/) - Robot Operating System
- [MoveIt](https://moveit.ros.org/) - Motion Planning Framework
- [OpenCV](https://opencv.org/) - Computer Vision Library
- [Python](https://www.python.org/) - Programming Language

---

**Note**: This program is pre-installed on the ArmPi FPV robot system and can be run directly. For detailed tutorials, please refer to the [Official Documentation](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/).
