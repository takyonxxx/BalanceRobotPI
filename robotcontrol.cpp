#include "robotcontrol.h"
#include <QDateTime>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <pigpio.h>  // Include pigpio library

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
    // Make sure ESCs are at neutral
    stopAllEscs();

    // Stop timers
    mainLoopTimer.stop();
    batteryTimer.stop();
    telemetryTimer.stop();

    // Note: pigpio termination is handled in main
}

bool RobotControl::initialize()
{
    std::cout << "Initializing RobotControl..." << std::endl;

    // Note: pigpio should already be initialized in main()
    // so we don't initialize it here again

    // Initialize LED pin
    gpioSetMode(PIN_LED, PI_OUTPUT);
    gpioWrite(PIN_LED, 1);  // Turn on LED

    // Initialize battery ADC pin
    gpioSetMode(PIN_BATTERY, PI_INPUT);

    // Initialize ESC PWM control - silently during initialization
    initEscPwm();

    // Initialize and calibrate MPU6050
    initMpu6050();

    // Perform initial battery check
    batteryTest();
    std::cout << "Battery voltage: " << getBatteryVoltage() << "V" << std::endl;

    // Start timers with adjusted timing for Raspberry Pi 5
    // Main loop at 10ms (100Hz) instead of 4ms (250Hz) to avoid overruns
    mainLoopTimer.start(10);       // Main loop every 10ms (100Hz)
    batteryTimer.start(1000);      // Battery check every 1s
    telemetryTimer.start(130);     // Telemetry every ~130ms

    std::cout << "Initialization complete. Place robot vertical to run." << std::endl;
    return true;
}

void RobotControl::powerOn()
{
    power = true;
    moveRobot(0.0f, 0.0f, 0.0f, 0.0f, 0);
    std::cout << "Robot powered on" << std::endl;
}

void RobotControl::powerOff()
{
    power = false;
    moveRobot(0.0f, 0.0f, 0.0f, 0.0f, 0);
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

// Unified robot movement control
bool RobotControl::moveRobot(float forward, float lateral, float yaw, float vertical, quint16 buttons)
{
    // Ensure power is on
    if (!power) {
        debugLog("Cannot move robot: Power is off");
        return false;
    }

    // Ensure battery is not discharged
    if (batteryDischarged) {
        debugLog("Cannot move robot: Battery is discharged");
        return false;
    }

    // Convert float values (-1.0 to 1.0) to throttle values (-100 to 100)
    int16_t forwardThrottle = static_cast<int16_t>(forward * 100.0f);
    int16_t lateralThrottle = static_cast<int16_t>(lateral * 100.0f);
    int16_t yawThrottle = static_cast<int16_t>(yaw * 100.0f);

    // Constrain values to valid range
    forwardThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), forwardThrottle));
    lateralThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), lateralThrottle));
    yawThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), yawThrottle));

    // For balance robot, we mostly care about forward/backward and rotation (yaw)
    // We'll use the yaw to adjust the differential between left and right motors

    // Calculate right and left throttle values
    int16_t rightThrottle = forwardThrottle - yawThrottle;
    int16_t leftThrottle = forwardThrottle + yawThrottle;

    // Constrain final throttle values
    rightThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), rightThrottle));
    leftThrottle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), leftThrottle));

    // Log movement if verbose output is enabled
    if (verboseOutput) {
        std::cout << "Moving robot: Forward=" << forwardThrottle
                  << ", Yaw=" << yawThrottle
                  << ", Right=" << rightThrottle
                  << ", Left=" << leftThrottle
                  << std::endl;
    }

    // Set ESC PWM values
    setEscPwm(rightThrottle, PIN_ESC_1);
    setEscPwm(leftThrottle, PIN_ESC_2);

    // Process button inputs if needed
    if (buttons & 0x0001) { // Example: Button 1 - Fast mode
        setFastMode(true);
    }
    if (buttons & 0x0002) { // Example: Button 2 - Normal mode
        setFastMode(false);
    }

    return true;
}

void RobotControl::testAllEscs(int durationSeconds)
{
    std::cout << "Testing ESCs for " << durationSeconds << " seconds" << std::endl;

    // Get current time
    auto startTime = std::chrono::steady_clock::now();
    auto endTime = startTime + std::chrono::seconds(durationSeconds);

    int16_t throttle = 0;
    int16_t direction = 5;

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

        // Print current throttle values for debugging
        std::cout << "ESC Test: Right = " << throttle << ", Left = " << -throttle
                  << " (PWM vals: " << mapThrottleToPwm(throttle) << ", " << mapThrottleToPwm(-throttle) << ")" << std::endl;

        // Small delay for smooth ramping
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

// Helper function to map throttle (-100 to 100) to PWM value (1000 to 2000)
int RobotControl::mapThrottleToPwm(int16_t throttle)
{
    // Constrain throttle value to -100 to 100
    throttle = std::max(static_cast<int16_t>(-100), std::min(static_cast<int16_t>(100), throttle));

    // Map throttle (-100 to 100) to ESC PWM range (1000 to 2000)
    // -100 => 1000, 0 => 1500, 100 => 2000
    return ESC_PWM_NEUTRAL + (throttle * (ESC_PWM_MAX - ESC_PWM_NEUTRAL) / 100);
}

void RobotControl::setEscPwm(int16_t throttle, int escPin)
{
    // Convert throttle to pulse width in microseconds
    int pulseWidth = mapThrottleToPwm(throttle);

    // Using our range of 20000, the pulse width value can be directly used as dutycycle
    // Since 1000-2000μs pulse in a 20ms period corresponds to 1000-2000 dutycycle in a 20000 range

    // Set PWM duty cycle directly using the microsecond value
    gpioPWM(escPin, pulseWidth);

    // Debug output
    if (verboseOutput) {
        std::cout << "ESC " << escPin << ": Throttle=" << throttle
                  << ", Pulse=" << pulseWidth << "μs, Duty="
                  << (pulseWidth * 100.0 / 20000.0) << "%" << std::endl;
    }
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
    static bool firstRun = true;
    static bool warnedLoopTime = false;

    auto startTime = QDateTime::currentMSecsSinceEpoch();

    // Skip first run to avoid unnecessary messages
    if (firstRun) {
        firstRun = false;
        return;
    }

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

        // Emit warning if loop time is too long (but only once)
        if (loopTimeMs > 8.0f && !warnedLoopTime) {
            emit loopOverrun(loopTimeMs);
            warnedLoopTime = true;

            // Adjust loop timing if possible
            std::cout << "Adjusting control loop timing for better performance..." << std::endl;
            mainLoopTimer.stop();
            mainLoopTimer.start(10); // Increase to 10ms to match system capabilities
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
    // Set pins as outputs
    gpioSetMode(PIN_ESC_1, PI_OUTPUT);
    gpioSetMode(PIN_ESC_2, PI_OUTPUT);

    // For hardware PWM, pigpio accepts ranges from 25-40000
    // and dutycycle from 0-range

    // Let's use a range of 20000 to match our 20ms period (50Hz)
    // This makes calculations easy:
    // - 1ms pulse = 1000 dutycycle value (5% duty)
    // - 1.5ms pulse = 1500 dutycycle value (7.5% duty)
    // - 2ms pulse = 2000 dutycycle value (10% duty)

    // Set PWM frequency to 50Hz
    gpioSetPWMfrequency(PIN_ESC_1, 50);
    gpioSetPWMfrequency(PIN_ESC_2, 50);

    // Set PWM range to 20000 (0-20000 corresponds to 0-100% duty cycle in 20ms period)
    // This gives us direct microsecond control (1 = 1μs in a 20ms period)
    gpioSetPWMrange(PIN_ESC_1, 20000);
    gpioSetPWMrange(PIN_ESC_2, 20000);

    // Set both ESCs to neutral (1500μs pulse width)
    gpioPWM(PIN_ESC_1, ESC_PWM_NEUTRAL);
    gpioPWM(PIN_ESC_2, ESC_PWM_NEUTRAL);

    std::cout << "ESC PWM initialized using pigpio hardware PWM" << std::endl;
    std::cout << "  Frequency: 50Hz (20ms period)" << std::endl;
    std::cout << "  Pulse range: 1000-2000μs (5-10% duty cycle)" << std::endl;
    std::cout << "  Neutral point: " << ESC_PWM_NEUTRAL << "μs (7.5% duty cycle)" << std::endl;

    // Small delay to allow ESCs to recognize the neutral signal
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    constexpr int numSamples = 255;
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
        gpioWrite(PIN_LED, i % 2);

        // Delay to get stable readings
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
    // Adjust for 10ms loop time instead of 4ms
    constexpr float deltaT = 0.010f;  // 10ms in seconds (was 4ms)
    constexpr float gyroScale = 65.536f;  // For +/- 500 deg/s with 16-bit values

    robotAngle += (gyroZ / gyroScale) * deltaT;

    // 2. Calculate angle from accelerometer
    float accAngle = atan2f(accelY, -accelX) * 57.2958f;  // Convert to degrees

    // 3. Combine using complementary filter
    // Adjust filter weight for 10ms timing
    const float adjustedGyroWeight = 0.985f;  // Slightly less weight for gyro at 10ms
    robotAngle = robotAngle * adjustedGyroWeight + accAngle * (1.0f - adjustedGyroWeight);

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
    float dInput = (3.0f * pidInput - 4.0f * lastPidInput + lastLastPidInput) / 3.0f;
    lastLastPidInput = lastPidInput;
    lastPidInput = pidInput;

    // Adjusted PID constants for 10ms loop time (vs original 4ms)
    const float adjustedKD = KD * (4.0f / 10.0f); // Scale Kd for 10ms loop time

    // Calculate PID output
    pidOutput = KP * pidInput + adjustedKD * dInput + KC * robotPosition + KV * robotSpeed;

    // Limit PID output
    if (pidOutput > PID_OUT_MAX) {
        pidOutput = PID_OUT_MAX;
    } else if (pidOutput < PID_OUT_MIN) {
        pidOutput = PID_OUT_MIN;
    }

    // Update robot speed by integrating acceleration
    // Adjust integration for 10ms vs 4ms
    robotSpeed += pidOutput * (10.0f / 4.0f);

    // Safety check - stop if speed is too large
    if (robotSpeed > 2000.0f || robotSpeed < -2000.0f) {
        vertical = false;
        return;
    }

    // Update robot position by integrating velocity and add joystick offset
    // Adjust integration for 10ms vs 4ms
    robotPosition += (robotSpeed * (10.0f / 4.0f)) - 1.5f * responseRate * joystickY;

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
        gpioWrite(PIN_LED, 1);  // LED on when battery discharged
    } else {
        if (batteryVoltage > RESET_VOLTAGE) {
            batteryDischarged = false;
        }

        // Flash LED when battery is good
        if (!batteryDischarged) {
            gpioWrite(PIN_LED, !gpioRead(PIN_LED));
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
}

// Helper function to log debug messages
void RobotControl::debugLog(const std::string& message)
{
    if (verboseOutput) {
        std::cout << "RobotControl: " << message << std::endl;
    }
}
