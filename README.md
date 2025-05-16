# BalanceRobot

A self-balancing robot project powered by Raspberry Pi 5, featuring precise motor control, Bluetooth connectivity, and a mobile app interface.

![BalanceRobot Frame Stl](https://github.com/takyonxxx/BalanceRobotPI/blob/main/stl/F2BNOQ7J839QTHL.stl)
![BalanceRobot Wheel Stl](https://github.com/takyonxxx/BalanceRobotPI/blob/main/stl/FDFVE76J839QTI2.stl)

## Overview

BalanceRobot is a two-wheeled self-balancing robot that uses PID control algorithms to maintain equilibrium while allowing for remote control via Bluetooth. The project consists of two major components:

1. **Raspberry Pi 5 Control System**: Handles motion sensing, balance control, and motor management
2. **Mobile App Interface**: Provides remote control and parameter adjustment through a user-friendly touch interface

## Features

- **Real-time Balance Control**: Uses MPU6050 IMU for precise angle measurement and PID control
- **High-precision PWM Generation**: Custom high-resolution PWM threads for precise ESC control
- **Bluetooth Low Energy (BLE) Connectivity**: Remote control and telemetry using GATT server
- **Touch-friendly Mobile Interface**: Intuitive control app optimized for touch screens
- **Parameter Tuning**: Adjust PID parameters in real-time for optimal performance
- **Battery Monitoring**: Built-in voltage monitoring with safety cutoff
- **Diagnostic Features**: Performance monitoring and test modes

## Hardware Requirements

- Raspberry Pi 5 (4 or 3 may work with timing adjustments)
- MPU6050 IMU sensor (connected via I2C)
- Two brushless motors with ESCs (Electronic Speed Controllers)
- Battery (3S LiPo recommended)
- Chassis with two wheels
- Mobile device with Bluetooth support (for remote control)

## Software Architecture

The project is built using Qt/C++ for the Raspberry Pi control system and features:

### Raspberry Pi Control System

- **RobotControl Class**: Core control system handling balance, motor control, and communication
- **GattServer**: Manages Bluetooth LE communication with the mobile app
- **High-Precision PWM**: Custom thread-based PWM generation for precise ESC control
- **MPU6050 Interface**: I2C communication with the motion sensor
- **PID Controller**: Implements the balance algorithm with adjustable parameters

### Mobile App

- **Touch-Friendly UI**: Optimized for smartphones with intuitive controls
- **Parameter Adjustment**: Real-time PID parameter tuning with intuitive +/- controls
- **Direction Controls**: Touch-based directional input for robot movement
- **Telemetry Display**: Real-time feedback of robot status and battery level

## Installation

### Raspberry Pi Setup

1. Install required dependencies:

```bash
sudo apt update
sudo apt install git cmake qtbase5-dev wiringpi libi2c-dev
```

2. Clone the repository:

```bash
git clone https://github.com/yourusername/balancerobot.git
cd balancerobot
```

3. Build the project:

```bash
mkdir build && cd build
cmake ..
make
```

4. Run the application:

```bash
sudo ./balancerobot
```

> **Note**: The application requires root privileges to access I2C and GPIO.

### Mobile App Setup

1. Install Qt on your development machine
2. Open the `mobile_app/BalanceRobotApp.pro` file in Qt Creator
3. Build and deploy to your Android or iOS device

## Usage

1. Power on the robot and ensure the Raspberry Pi boots properly
2. Launch the control application on the Raspberry Pi
3. Open the mobile app and connect to the robot via Bluetooth
4. Place the robot in an upright position to activate balance mode
5. Use the directional controls on the app to move the robot
6. Adjust PID parameters as needed for optimal balance performance

## PID Tuning Guide

The robot's balance performance can be optimized by adjusting the following parameters:

- **P (Proportional)**: Controls the response to the current angle error
- **D (Derivative)**: Controls the response to the rate of change of the angle
- **C (Position)**: Controls the drift correction
- **V (Velocity)**: Controls the damping of movement
- **AC (Angle Correction)**: Fine-tunes the neutral angle

Typical starting values:
- P: 0.1-0.5
- D: 3.0-10.0
- C: 0.0001-0.001
- V: 0.01-0.05
- AC: 0.0-5.0

## Troubleshooting

Common issues and solutions:

1. **Robot doesn't balance**: Ensure MPU6050 is properly connected and calibrated
2. **Motors don't respond**: Check ESC connections and PWM signal generation
3. **Unable to connect via Bluetooth**: Verify Bluetooth service is running and permissions are set
4. **Poor balance performance**: Adjust PID parameters starting with P and D values
5. **Loop overrun warnings**: Increase mainLoop timing to accommodate system capabilities

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Thanks to the Raspberry Pi Foundation for the amazing hardware
- Inspiration from various self-balancing robot projects
- Qt framework for the cross-platform development capabilities
