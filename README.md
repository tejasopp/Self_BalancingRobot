# Self-Balancing Robot

This project is a self-balancing robot that uses stepper motors, an ESP32 microcontroller, stepper motor drivers, and an IMU sensor. The robot maintains its balance by employing a PID control algorithm to adjust the motors based on the feedback from the IMU sensor.

## Table of Contents
1. [Introduction](#introduction)
2. [Components](#components)
3. [Circuit Diagram](#circuit-diagram)
4. [Software](#software)
5. [PID Control](#pid-control)
6. [Assembly](#assembly)
7. [Usage](#usage)
8. [References](#References)

## Introduction
The self-balancing robot is an advanced project that demonstrates the use of control systems, sensors, and microcontrollers. It can be used as a platform for further experiments with robotics and control theory.

## Components
- ESP32 Development Board
- IMU Sensor ( MPU6050)
- Stepper Motors (x2)
- Stepper Motor Drivers (x2, A4988)
- LiPo Battery
- Battery Pack
- Chassis for mounting components
- Connecting wires and cables
- Breadboard or PCB for connections

## Circuit Diagram
![image](https://github.com/user-attachments/assets/cd28a150-c388-4e82-ad8a-3c7823c4e201)

## Software
### Prerequisites
- Arduino IDE
- ESP32 Board Package
- MPU6050 Library
- PID Library

### Installation
1. **Arduino IDE Setup**
   - Install the ESP32 board package in the Arduino IDE.
   - Install the MPU6050 library and PID library from the Arduino Library Manager.

### Code
Upload the provided code to the ESP32 using Arduino IDE.

## PID Control
The PID control algorithm is used to adjust the motor speeds to maintain balance. The parameters (Kp, Ki, Kd) need to be tuned for optimal performance.

## Assembly
1. **Mount the Components**
   - Secure the ESP32, stepper motors, motor drivers, and IMU sensor to the chassis.
   - Connect the stepper motors to the motor drivers.
   - Wire the motor drivers to the ESP32 according to the circuit diagram.
   - Connect the IMU sensor to the ESP32 via I2C.

2. **Power Supply**
   - Ensure the power supply is suitable for the stepper motors and the ESP32.
   - Connect the battery pack and verify all connections.

## Usage
1. **Uploading the Code**
   - Upload the provided code to the ESP32 using Arduino IDE.
   
2. **Balancing the Robot**
   - Place the robot on a flat surface.
   - Power on the robot.
   - The robot should start balancing. Adjust the PID parameters if necessary.

## References
- https://gitlab.com/kloppertje/balancingrobot/-/tree/ps3control

---
