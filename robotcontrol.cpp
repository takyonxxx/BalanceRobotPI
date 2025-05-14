#include "robotcontrol.h"
#include <QDateTime>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>

RobotControl::RobotControl(QObject *parent) : QObject(parent),
    power(false),
    vertical(false),
    batteryDischarged(false),
    robotAngle(0.0f),
    robotSpeed(0.0f),
    robotPosition(0),
    batteryVoltage(0.0f),
    loopTimeMs(0.0f),
    joystickX(0),
    joystickY(0),
    responseRate(1),
    pidInput(0.0f),
    lastPidInput(0.0f),
    lastLastPidInput(0.0f),
    pidOutput(0.0f),
    accelXOffset(0),
    accelYOffset(0),
    gyroZOffset(0),
    gyroFilterEnabled(true),
    filterCount(0),
    verboseOutput(false)  // Start with silent operation
{
    // Initialize the gyro filter array to zero
    gyroZFilter.fill(0);

    // Setup timers
    connect(&mainLoopTimer, &QTimer::timeout, this, &RobotControl::mainLoop);
    connect(&batteryTimer, &QTimer::timeout, this, &RobotControl::batteryCheckTimer);
    connect(&telemetryTimer, &QTimer::timeout, this, &RobotControl::sendDataTimer);
}

RobotControl::~RobotControl()
{
    // Enable verbose output for shutdown
    verboseOutput = true;

    // Make sure ESCs are at neutral
    stopAllEscs();

    // Stop timers
    mainLoopTimer.stop();
    batteryTimer.stop();
    telemetryTimer.stop();
}

bool RobotControl::initialize()
{
    std::cout << "Initializing RobotControl..." << std::endl;

    // Initialize wiringPi library
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failed to initialize wiringPi" << std::endl;
        return false;
    }

    // Initialize LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialize battery ADC pin
    pinMode(PIN_BATTERY, INPUT);

    // Initialize ESC PWM control - silently during initialization
    initEscPwm();

    // Initialize and calibrate MPU6050
    initMpu6050();

    // Perform initial battery check
    batteryTest();
    std::cout << "Battery voltage: " << getBatteryVoltage() << "V" << std::endl;

    // Start timers
    mainLoopTimer.start(4);        // Main loop every 4ms
    batteryTimer.start(1000);      // Battery check every 1s
    telemetryTimer.start(130);     // Telemetry every ~130ms

    std::cout << "Initialization complete. Place robot vertical to run." << std::endl;
    return true;
}

void RobotControl::powerOn()
{
    power = true;
    verboseOutput = true;  // Enable verbose output after user powers on
    std::cout << "Robot powered on" << std::endl;
}

void RobotControl::powerOff()
{
    power = false;
    verboseOutput = true;  // Enable verbose output before stopping ESCs
    stopAllEscs();
    std::cout << "Robot powered off" << std::endl;
}

void RobotControl::setFastMode(bool enable)
{
    responseRate = enable ? 2 : 1;
    std::cout << "Fast mode: " << (enable ? "enabled" : "disabled") << std::endl;
}

void RobotControl::setJoystickInput(int16_t x, int16_t y)
{
    // Limit joystick range to -100 to 100
    joystickX = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), x));
    joystickY = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), y));

    // If MPU6050 is not working, use direct ESC control instead
    // This allows joystick control even without the balance functionality
    if (!vertical && power && !batteryDischarged) {
        // Map joystick Y to forward/backward
        int16_t throttleValue = joystickY;

        // Map joystick X to steering (differential drive)
        int16_t rightThrottle = throttleValue - joystickX/2;
        int16_t leftThrottle = throttleValue + joystickX/2;

        // Apply direct control to ESCs
        directEscControl(rightThrottle, leftThrottle);
    }
}

void RobotControl::testAllEscs(int durationSeconds)
{
    verboseOutput = true;  // Enable verbose output during testing
    std::cout << "Testing ESCs for " << durationSeconds << " seconds" << std::endl;

    // Get current time
    auto startTime = std::chrono::steady_clock::now();
    auto endTime = startTime + std::chrono::seconds(durationSeconds);

    int16_t throttle = 0;
    int16_t direction = 10;

    // Slowly ramp up and down ESC power in both directions to test
    while (std::chrono::steady_clock::now() < endTime) {
        // Update throttle (oscillate between -100 and 100)
        throttle += direction;
        if (throttle > 100 || throttle < -100) {
            direction = -direction;
        }

        // Apply throttle to both ESCs
        setEscPwm(throttle, PIN_ESC_1);
        setEscPwm(-throttle, PIN_ESC_2);  // Opposite direction for the second ESC

        // Small delay for smooth ramping
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Set ESCs to neutral when done
    stopAllEscs();
    std::cout << "ESC test complete" << std::endl;
}

void RobotControl::stopAllEscs()
{
    // Set ESCs to neutral position
    setEscPwm(0, PIN_ESC_1);
    setEscPwm(0, PIN_ESC_2);

    // Only print message if verbose output is enabled
    if (verboseOutput) {
        std::cout << "ESCs stopped" << std::endl;
    }
}

// Add a simple manual ESC control method that bypasses balance control
void RobotControl::directEscControl(int16_t rightThrottle, int16_t leftThrottle)
{
    // Limit throttle to -100 to 100
    rightThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), rightThrottle));
    leftThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), leftThrottle));

    // Set ESC PWM values directly
    setEscPwm(rightThrottle, PIN_ESC_1);
    setEscPwm(leftThrottle, PIN_ESC_2);

    // Only print message if verbose output is enabled
    if (verboseOutput) {
        std::cout << "Direct ESC control: Right=" << rightThrottle << ", Left=" << leftThrottle << std::endl;
    }
}

void RobotControl::setEscPwm(int16_t throttle, int escPin)
{
    // Constrain throttle value to -100 to 100
    throttle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), throttle));

    // Map throttle (-100 to 100) to ESC PWM range (1000 to 2000)
    // -100 => 1000, 0 => 1500, 100 => 2000
    int pwmValue = ESC_PWM_NEUTRAL + (throttle * (ESC_PWM_MAX - ESC_PWM_NEUTRAL) / 100);

    // Send PWM signal to ESC
    softPwmWrite(escPin, pwmValue);
}

float RobotControl::getBatteryVoltage() const
{
    return batteryVoltage;
}

bool RobotControl::isBatteryDischarged() const
{
    return batteryDischarged;
}

float RobotControl::getLoopTime() const
{
    return loopTimeMs;
}

void RobotControl::mainLoop()
{
    auto startTime = QDateTime::currentMSecsSinceEpoch();

    // Calculate the robot's angle
    calculateAngle();

    // Run PID control if powered and battery not discharged
    if (power && !batteryDischarged) {
        calculatePid();
    } else {
        // Just reset control variables, don't call stopAllEscs()
        lastLastPidInput = 0.0f;
        lastPidInput = 0.0f;
        pidOutput = 0.0f;
        robotSpeed = 0.0f;
        robotPosition = 0;
    }

    // Calculate loop time
    auto endTime = QDateTime::currentMSecsSinceEpoch();
    auto elapsed = endTime - startTime;

    // Track maximum loop time
    if (elapsed > loopTimeMs) {
        loopTimeMs = elapsed;

        // Emit warning if loop time is too long
        if (loopTimeMs > 4.0f) {
            emit loopOverrun(loopTimeMs);
        }
    }
}

void RobotControl::batteryCheckTimer()
{
    batteryTest();

    if (batteryDischarged) {
        emit batteryLow();
    }
}

void RobotControl::sendDataTimer()
{
    emit telemetryData(getBatteryVoltage(), getLoopTime());

    // Reset loop time after reporting
    loopTimeMs = 0.0f;
}

void RobotControl::initEscPwm()
{
    // Initialize ESC PWM pins with neutral value
    softPwmCreate(PIN_ESC_1, ESC_PWM_NEUTRAL, ESC_PWM_RANGE);
    softPwmCreate(PIN_ESC_2, ESC_PWM_NEUTRAL, ESC_PWM_RANGE);

    std::cout << "ESC PWM initialized" << std::endl;

    // Small delay to allow ESCs to recognize the neutral signal
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void RobotControl::initMpu6050()
{
    // Initialize I2C
    std::cout << "Initializing I2C devices..." << std::endl;
    mpu.initialize();

    // Verify connection
    std::cout << "Testing MPU6050 connection..." << std::endl;
    if (!mpu.testConnection()) {
        std::cerr << "MPU6050 connection failed!" << std::endl;
        std::cerr << "Check your I2C wiring and make sure MPU6050 is powered properly" << std::endl;
        std::cerr << "The program will continue but balance control will be disabled" << std::endl;
        vertical = false;  // Disable balance control
        return;
    }

    std::cout << "MPU6050 connection successful" << std::endl;

    // Configure MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);    // +/- 4g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);     // +/- 500 degrees/sec

    // Calibrate gyroscope
    std::cout << "Calibrating gyroscope. Place robot on a flat surface..." << std::endl;
    calibrateGyro();
}

void RobotControl::calibrateGyro()
{
    constexpr int numSamples = 1024;
    int32_t gyroZOffsetSum = 0;
    int16_t accelX, accelY, gyroZ;

    // Store current filter state and disable during calibration
    bool tempFilterState = gyroFilterEnabled;
    gyroFilterEnabled = false;

    // Gather and average 1024 readings
    for (int i = 0; i < numSamples; i++) {
        if (readMpuData(accelX, accelY, gyroZ)) {
            gyroZOffsetSum += gyroZ;
        }

        // Flash LED during calibration
        digitalWrite(PIN_LED, i % 2);

        // Delay to get stable readings
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Calculate average
    gyroZOffset = gyroZOffsetSum / numSamples;

    // Restore filter state
    gyroFilterEnabled = tempFilterState;

    std::cout << "Gyro Z-axis offset: " << gyroZOffset << std::endl;
}

bool RobotControl::readMpuData(int16_t &accelX, int16_t &accelY, int16_t &gyroZ)
{
    // Temporary variables for full motion data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Check if MPU initialized properly
    static bool lastFailed = false;

    try {
        // Read raw measurements from MPU6050
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // If we get here without an error, reset the failure flag
        if (lastFailed) {
            lastFailed = false;
        }
    }
    catch (...) {
        // Handle any errors during MPU6050 reading
        if (!lastFailed) {
            std::cerr << "Error reading from MPU6050" << std::endl;
            lastFailed = true;
        }
        return false;
    }

    // Assign to output parameters
    accelX = ax;
    accelY = ay;
    gyroZ = gz;

    return true;
}

void RobotControl::calculateAngle()
{
    static bool warnedMpu = false;
    int16_t accelX, accelY, gyroZ;

    // Read raw data from MPU6050
    if (!readMpuData(accelX, accelY, gyroZ)) {
        // Only print MPU warning once to avoid spamming
        if (!warnedMpu) {
            std::cerr << "Warning: Failed to read MPU6050 data" << std::endl;
            warnedMpu = true;
        }
        // Set vertical to false to prevent control loop from running
        vertical = false;
        return;
    }

    // Apply filter to gyro Z readings if enabled
    if (gyroFilterEnabled) {
        // Simple moving average filter
        gyroZFilter[filterCount] = gyroZ;
        filterCount = (filterCount + 1) % gyroZFilter.size();

        int32_t gyroZFiltered = 0;
        for (const auto& value : gyroZFilter) {
            gyroZFiltered += value;
        }
        gyroZ = gyroZFiltered / gyroZFilter.size();
    }

    // Add offsets
    accelX += accelXOffset;
    accelY += accelYOffset;
    gyroZ += gyroZOffset;

    // Complementary filter
    // 1. Integrate gyroscope to get angle change
    // The time constant is based on the 4ms loop time
    constexpr float deltaT = 0.004f;  // 4ms in seconds
    constexpr float gyroScale = 65.536f;  // For +/- 500 deg/s with 16-bit values

    robotAngle += (gyroZ / gyroScale) * deltaT;

    // 2. Calculate angle from accelerometer
    float accAngle = atan2f(accelY, -accelX) * 57.2958f;  // Convert to degrees

    // 3. Combine using complementary filter
    robotAngle = robotAngle * GYRO_WEIGHT + accAngle * (1.0f - GYRO_WEIGHT);

    // Check if robot is vertical
    if (robotAngle > 50.0f || robotAngle < -50.0f) {
        vertical = false;
    }
    if (robotAngle < 1.0f && robotAngle > -1.0f) {
        vertical = true;
    }
}

void RobotControl::calculatePid()
{
    if (!vertical) {
        // stopAllEscs() removed here to prevent messages
        // ESCs will still be at neutral from the last command

        lastLastPidInput = 0.0f;
        lastPidInput = 0.0f;
        pidOutput = 0.0f;
        robotSpeed = 0.0f;
        robotPosition = 0;
        return;
    }

    // Calculate PID input (error)
    pidInput = robotAngle;

    // Calculate derivative term (using finite difference)
    float dInput = (3.0f * pidInput - 4.0f * lastPidInput + lastLastPidInput);
    lastLastPidInput = lastPidInput;
    lastPidInput = pidInput;

    // Calculate PID output
    pidOutput = KP * pidInput + KD * dInput + KC * robotPosition + KV * robotSpeed;

    // Limit PID output
    if (pidOutput > PID_OUT_MAX) {
        pidOutput = PID_OUT_MAX;
    } else if (pidOutput < PID_OUT_MIN) {
        pidOutput = PID_OUT_MIN;
    }

    // Update robot speed by integrating acceleration
    robotSpeed += pidOutput;

    // Safety check - stop if speed is too large
    if (robotSpeed > 2000.0f || robotSpeed < -2000.0f) {
        vertical = false;
        return;
    }

    // Update robot position by integrating velocity and add joystick offset
    robotPosition += robotSpeed - 1.5f * responseRate * joystickY;

    // Calculate right and left motor throttle values (from -100 to 100)
    int16_t rightThrottle = pidOutput * 100.0f / PID_OUT_MAX;
    int16_t leftThrottle = pidOutput * 100.0f / PID_OUT_MAX;

    // Adjust motors to turn robot based on joystick X input
    float rotationFactor = 0.3f * joystickX;
    rightThrottle += rotationFactor;
    leftThrottle -= rotationFactor;

    // Limit throttle to -100 to 100
    rightThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), rightThrottle));
    leftThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), leftThrottle));

    // Set ESC PWM values - verboseOutput controls whether we see messages
    setEscPwm(rightThrottle, PIN_ESC_1);
    setEscPwm(leftThrottle, PIN_ESC_2);
}

void RobotControl::batteryTest()
{
    // Read battery voltage
    batteryVoltage = readBatteryVoltage();

    // Check battery state
    if (batteryVoltage < DISCHARGE_VOLTAGE) {
        batteryDischarged = true;
        digitalWrite(PIN_LED, HIGH);  // LED on when battery discharged
    } else {
        if (batteryVoltage > RESET_VOLTAGE) {
            batteryDischarged = false;
        }

        // Flash LED when battery is good
        if (!batteryDischarged) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        }
    }
}

float RobotControl::readBatteryVoltage()
{
    // For Raspberry Pi, we need to use an ADC connected via I2C or SPI
    // This is a simplified implementation that assumes a voltage divider
    // connected to one of the analog inputs of an external ADC

    // For demonstration purposes, returning a fixed value
    // In a real implementation, you would read from an ADC

    // Fixed voltage for testing (11.1V battery - 3S LiPo)
    return 11.1f;

    /*
    // If you have an ADC connected, implement your specific ADC reading code here
    // Example for ADS1115 or similar I2C ADC would go here

    // Example using an analog read from a proper ADC implementation
    int adcValue = analogRead(PIN_BATTERY);
    float voltage = adcValue * (3.3f / 1023.0f) * 9.33f;  // Example scale factor for voltage divider
    return voltage;
    */
}
