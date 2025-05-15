#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QObject>
#include <QTimer>
#include <cmath>
#include <iostream>
#include <array>
#include <vector>

// External libraries
#include "i2cdev.h"
#include "mpu6050.h"
// No wiringPi include - using pigpio instead

class RobotControl : public QObject
{
    Q_OBJECT

public:
    explicit RobotControl(QObject *parent = nullptr);
    virtual ~RobotControl();

    // Initialize hardware and sensors
    bool initialize();

    // Control methods
    void powerOn();
    void powerOff();
    void setFastMode(bool enable);
    void setJoystickInput(int16_t x, int16_t y);
    void testAllEscs(int durationSeconds);
    void stopAllEscs();
    void directEscControl(int16_t rightThrottle, int16_t leftThrottle);

    // ESC PWM Control
    void setEscPwm(int16_t throttle, int escPin);
    int mapThrottleToPwm(int16_t throttle);

    // Battery information
    float getBatteryVoltage() const;
    bool isBatteryDischarged() const;

    // Loop timing information
    float getLoopTime() const;

    // ESC PWM Pins - Made public so they can be directly referenced
    static constexpr int PIN_ESC_1 = 18;  // First ESC PWM Pin (GPIO 18, Pin 12)
    static constexpr int PIN_ESC_2 = 12;  // Second ESC PWM Pin (GPIO 12, Pin 32)

private slots:
    void mainLoop();             // Main control loop
    void batteryCheckTimer();    // Battery check timer
    void sendDataTimer();        // Send telemetry data

signals:
    void telemetryData(float batteryVoltage, float loopTime);
    void batteryLow();
    void loopOverrun(float actualTime);

private:
    // GPIO Pin Configuration
    static constexpr int PIN_LED = 4;     // Status LED Pin 7
    static constexpr int PIN_BATTERY = 0; // Battery voltage ADC Pin

    // PWM Configuration for ESC
    static constexpr int ESC_PWM_FREQUENCY = 50;   // 50 Hz (20ms period) for standard ESCs
    static constexpr int ESC_PWM_MIN = 1000;       // Minimum pulse width (1000μs = 1ms)
    static constexpr int ESC_PWM_NEUTRAL = 1500;   // Neutral pulse width (1500μs = 1.5ms)
    static constexpr int ESC_PWM_MAX = 2000;       // Maximum pulse width (2000μs = 2ms)

    // Battery Constants
    static constexpr float DISCHARGE_VOLTAGE = 3.0f * 3.0f; // 3 cells at 3.0V each
    static constexpr float RESET_VOLTAGE = 3.0f * 3.7f;     // 3 cells at 3.7V each

    // Balance Robot PID Constants
    static constexpr float KC = 0.0002f;  // Position feedback gain
    static constexpr float KV = 0.02f;    // Velocity feedback gain
    static constexpr float KP = 0.2f;     // Angle gain
    static constexpr float KD = 0.025f / (2.0f * 0.004f); // Angle velocity gain
    static constexpr float PID_OUT_MAX = 100.0f;
    static constexpr float PID_OUT_MIN = -100.0f;
    static constexpr float GYRO_WEIGHT = 0.996f;

    // Private methods
    void initEscPwm();
    void initMpu6050();
    void calibrateGyro();
    void calculateAngle();
    void calculatePid();
    void batteryTest();
    float readBatteryVoltage();

    // Helper method to read MPU6050 data
    bool readMpuData(int16_t &accelX, int16_t &accelY, int16_t &gyroZ);

    // Sensor and state variables
    MPU6050 mpu;                  // MPU6050 instance
    QTimer mainLoopTimer;         // Timer for main control loop
    QTimer batteryTimer;          // Timer for battery checks
    QTimer telemetryTimer;        // Timer for telemetry

    bool power;                   // Robot power state
    bool vertical;                // Is robot vertical enough to run
    bool batteryDischarged;       // Battery state

    float robotAngle;             // Current robot angle
    float robotSpeed;             // Current robot speed
    int32_t robotPosition;        // Offset from correct position

    float batteryVoltage;         // Battery voltage
    float loopTimeMs;             // Time taken by main loop

    int16_t joystickX;            // Joystick X input (-100 to 100)
    int16_t joystickY;            // Joystick Y input (-100 to 100)
    uint8_t responseRate;         // Joystick response multiplier

    // PID variables
    float pidInput;
    float lastPidInput;
    float lastLastPidInput;
    float pidOutput;

    // IMU calibration values
    int16_t accelXOffset;
    int16_t accelYOffset;
    int16_t gyroZOffset;
    bool gyroFilterEnabled;
    std::array<int32_t, 32> gyroZFilter;
    uint8_t filterCount;

    bool verboseOutput;           // Controls console output
};

#endif // ROBOTCONTROL_H
