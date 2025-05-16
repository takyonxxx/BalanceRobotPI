# BalanceRobot

A self-balancing robot project powered by Raspberry Pi 5, featuring precise motor control with 2208 brushless gimbal motors, bidirectional ESCs, Bluetooth connectivity, and a mobile app interface.

## 3D Models

You can view and interact with the 3D models directly in GitHub. The 2208 brushless gimbal motors will be placed inside the wheels for a compact, integrated design:

- [View BalanceRobot Frame STL](https://github.com/takyonxxx/BalanceRobotPI/blob/main/stl/F2BNOQ7J839QTHL.stl)
- [View BalanceRobot Wheel STL](https://github.com/takyonxxx/BalanceRobotPI/blob/main/stl/FDFVE76J839QTI2.stl)

## Overview

BalanceRobot is a two-wheeled self-balancing robot that uses PID control algorithms to maintain equilibrium while allowing for remote control via Bluetooth. The project consists of two major components:

1. **Raspberry Pi 5 Control System**: Handles motion sensing, balance control, and motor management
2. **Mobile App Interface**: Provides remote control and parameter adjustment through a user-friendly touch interface

## Features

- **Real-time Balance Control**: Uses MPU6050 IMU for precise angle measurement and PID control
- **High-precision PWM Generation**: Custom high-resolution PWM threads for precise ESC control
- **Powerful Motor System**: 2X 2208 brushless gimbal motors with bidirectional ESCs for responsive movement
- **Bluetooth Low Energy (BLE) Connectivity**: Remote control and telemetry using GATT server
- **Touch-friendly Mobile Interface**: Intuitive control app optimized for touch screens
- **Parameter Tuning**: Adjust PID parameters in real-time for optimal performance
- **Battery Monitoring**: Built-in voltage monitoring with safety cutoff
- **Diagnostic Features**: Performance monitoring and test modes

## Hardware Requirements

- Raspberry Pi 5 (4 or 3 may work with timing adjustments)
- MPU6050 IMU sensor (connected via I2C)
- 2X 2208 brushless gimbal motors
- Bidirectional ESCs (Electronic Speed Controllers)
- Battery (3S LiPo recommended)
- 3D printed chassis with two wheels
- Mobile device with Bluetooth support (for remote control)

## Motor & ESC Setup

The project uses 2208 brushless gimbal motors, which are ideal for self-balancing robots due to their:
- High torque at low RPM
- Precise control
- Compact design
- Low vibration

The bidirectional ESCs provide:
- Forward and reverse control
- Smooth acceleration and deceleration
- Compatible with standard PWM signal from Raspberry Pi
- Quick response time essential for balance corrections

## Connections and Pinouts

### Raspberry Pi 5 GPIO Connections

| Component | Raspberry Pi Pin | Description |
|-----------|------------------|-------------|
| **MPU6050** | | |
| VCC | 3.3V (Pin 1) | Power for MPU6050 |
| GND | GND (Pin 6) | Ground |
| SCL | GPIO 3 (Pin 5) | I2C Clock |
| SDA | GPIO 2 (Pin 3) | I2C Data |
| | | |
| **ESC1 (Right Motor)** | | |
| Signal | GPIO 18 (Pin 12) | PWM Control Signal |
| VCC | External Power Only | Do NOT connect to Raspberry Pi |
| GND | GND (Pin 14) | Common ground |
| | | |
| **ESC2 (Left Motor)** | | |
| Signal | GPIO 19 (Pin 35) | PWM Control Signal |
| VCC | External Power Only | Do NOT connect to Raspberry Pi |
| GND | GND (Pin 39) | Common ground |
| | | |
| **Status LED** | | |
| LED+ | GPIO 22 (Pin 15) | Status indicator |
| LED- | GND (Pin 34) | via appropriate resistor |

### MPU6050 Orientation

For proper balance control, the MPU6050 should be mounted with:
- X-axis aligned parallel to the wheels (pointing forward)
- Y-axis perpendicular to the wheels (pointing to the side)
- Z-axis pointing upward

### Power Distribution

- Power the Raspberry Pi via its USB-C power input
- Power the ESCs and motors with a separate 3S LiPo battery (11.1V) 
- Ensure common ground between Raspberry Pi and ESC power circuits
- Use appropriate voltage regulators/BECs if powering everything from a single battery

### Wiring Diagram

```
                           +---------------+
                           | Raspberry Pi 5|
                           +---------------+
                           | GPIO2  - SDA  |----+
                           | GPIO3  - SCL  |---+|
                           | GPIO18 - ESC1 |--+ ||
                           | GPIO19 - ESC2 |-+ |||
                           | GPIO22 - LED  |+ ||||
                           | GND           |-|-||||
                           | 3.3V          |--|-|||
                           +---------------+  | |||
                                              | |||
+----------+                  +----------+    | |||
| 3S LiPo  |                  | MPU6050  |<---|-||+
| Battery  |                  +----------+    | ||
+----------+                                  | ||
      |                      +------------+   | ||
      +---+           +------| Right ESC  |<--|-|+
          |           |      +------------+   | |
      +---+-----------+--+                    | |
      |      BEC*     |  |   +------------+   | |
      | (if required) |  +---| Left ESC   |<--|-+
      +--------------+      +------------+    |
                                              |
                            +------------+    |
                            | Status LED |<---+
                            +------------+
```
*BEC: Battery Eliminator Circuit (voltage regulator)

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
# Install build essentials
sudo apt-get install build-essential devscripts

# Install Qt6 development packages
sudo apt install qt6-base-dev qt6-base-dev-tools
sudo apt install qt6-connectivity-dev libqt6bluetooth6 qt6-base-dev

# Setup qmake symlink
sudo ln -sf /usr/lib/qt6/bin/qmake6 /usr/local/bin/qmake

# Install I2C development packages
sudo apt install libi2c-dev i2c-tools

# Install WiringPi from source
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
sudo ./build
cd ..
```

2. Clone the repository:

```bash
git clone https://github.com/takyonxxx/BalanceRobotPI.git
cd BalanceRobotPI
```

3. Build the project:

```bash
qmake
make
```

4. Run the application:

```bash
sudo ./balancerobot
```

> **Note**: The application requires root privileges to access I2C and GPIO.

### Mobile App Setup

1. Install Qt on your development machine
2. Open the `mobile_app/BalanceRobotRemote.pro` file in Qt Creator qt6.9 Clang arm64-v8a
3. Build and deploy to your Android or iOS device

## Usage

1. Power on the robot and ensure the Raspberry Pi boots properly
2. Launch the control application on the Raspberry Pi
3. Open the mobile app and connect to the robot via Bluetooth
4. Place the robot in an upright position to activate balance mode
5. Arm device and Use the directional controls on the app to move the robot
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

## License

This project is licensed under the MIT License - see the LICENSE file for details.
