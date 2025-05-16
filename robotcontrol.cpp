#include "robotcontrol.h"
#include <QDateTime>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "initwiringpi.h"

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
    pwmRunning(false),
    esc1PulseWidth(ESC_PWM_NEUTRAL),
    esc2PulseWidth(ESC_PWM_NEUTRAL),
    verboseOutput(false)
{
    // Initialize the gyro filter array to zero
    gyroZFilter.fill(0);

    // Initialize wiringPi before creating the RobotControl instance
    if (initWiringPi() < 0) {
        std::cerr << "Failed to initialize wiringPi! Exiting." << std::endl;
        return;
    }

    // Setup timers
    connect(&mainLoopTimer, &QTimer::timeout, this, &RobotControl::mainLoop);
    connect(&batteryTimer, &QTimer::timeout, this, &RobotControl::batteryCheckTimer);
    connect(&telemetryTimer, &QTimer::timeout, this, &RobotControl::sendDataTimer);

    gattServer = GattServer::getInstance();
    if (gattServer)
    {
        qDebug() << "Starting gatt service";
        QObject::connect(gattServer, &GattServer::connectionState, this, &RobotControl::onConnectionStatedChanged);
        QObject::connect(gattServer, &GattServer::dataReceived, this, &RobotControl::onDataReceived);
        gattServer->startBleService();
    }

    // Initialize robot control
    if (!initialize()) {
        std::cerr << "Failed to initialize robot control! Exiting." << std::endl;
        return;
    }
}

RobotControl::~RobotControl()
{
    // Make sure ESCs are at neutral
    stopAllEscs();

    // Stop timers
    mainLoopTimer.stop();
    batteryTimer.stop();
    telemetryTimer.stop();

    // Stop PWM threads
    pwmRunning = false;
    if (pwmThread1.joinable()) {
        pwmThread1.join();
    }
    if (pwmThread2.joinable()) {
        pwmThread2.join();
    }

    delete gattServer;
}

void RobotControl::pwmGeneratorThread(int pin, std::atomic<int>& pulseWidth)
{
    // Period for 50Hz = 20,000 microseconds
    const int periodUs = 20000;

    // Measure sleep overhead once at startup
    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    end = std::chrono::high_resolution_clock::now();
    int overhead = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() - 100;
    overhead = std::max(0, overhead); // Ensure non-negative

    std::cout << "PWM thread for pin " << pin << " started. Sleep overhead: " << overhead << "μs" << std::endl;

    // Set thread to high priority
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(this_thread, SCHED_FIFO, &params);

    while (pwmRunning) {
        // Get start time for this cycle
        auto cycleStart = std::chrono::high_resolution_clock::now();

        // Get current pulse width
        int width = pulseWidth.load();

        // Generate pulse
        digitalWrite(pin, HIGH);

        // High precision sleep for pulse width
        if (width > overhead) {
            std::this_thread::sleep_for(std::chrono::microseconds(width - overhead));
        }

        // Get precise time after sleep
        auto pulseEnd = std::chrono::high_resolution_clock::now();
        auto elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(
                             pulseEnd - cycleStart).count();

        // Adjust timing if we slept too long
        if (elapsedUs < width) {
            // Spin-wait for the remaining time
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::high_resolution_clock::now() - cycleStart).count() < width) {
                // Busy wait
            }
        }

        // Set pin low at exactly the right time
        digitalWrite(pin, LOW);

        // Calculate time spent for the pulse
        auto pulseTime = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::high_resolution_clock::now() - cycleStart).count();

        // Sleep for the remainder of the period
        int remainingTime = periodUs - pulseTime;
        if (remainingTime > overhead) {
            std::this_thread::sleep_for(std::chrono::microseconds(remainingTime - overhead));
        }

        // Spin-wait to ensure accurate timing for next cycle
        while (std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - cycleStart).count() < periodUs) {
            // Busy wait
        }
    }
}

bool RobotControl::initialize()
{
    std::cout << "Initializing RobotControl..." << std::endl;

    // Note: wiringPi should already be initialized in main()
    // so we don't initialize it here again

    // Initialize LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // Turn on LED

    // Initialize battery ADC pin
    pinMode(PIN_BATTERY, INPUT);

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
    pinMode(PIN_ESC_1, OUTPUT);
    pinMode(PIN_ESC_2, OUTPUT);

    // Set both pins low initially
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);

    // Set initial pulse widths
    esc1PulseWidth = ESC_PWM_NEUTRAL;
    esc2PulseWidth = ESC_PWM_NEUTRAL;

    std::cout << "Initializing PWM generation for ESCs..." << std::endl;
    std::cout << "  Target frequency: 50Hz (20ms period)" << std::endl;
    std::cout << "  Pulse range: 1000-2000μs (5-10% duty cycle)" << std::endl;
    std::cout << "  Neutral point: " << ESC_PWM_NEUTRAL << "μs (7.5% duty cycle)" << std::endl;

    // Start PWM threads
    pwmRunning = true;
    pwmThread1 = std::thread(&RobotControl::pwmGeneratorThread, this, PIN_ESC_1, std::ref(esc1PulseWidth));
    pwmThread2 = std::thread(&RobotControl::pwmGeneratorThread, this, PIN_ESC_2, std::ref(esc2PulseWidth));

    // Small delay to allow ESCs to recognize the neutral signal
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "ESC PWM initialization complete" << std::endl;
}

void RobotControl::setEscPwm(int16_t throttle, int escPin)
{
    // Convert throttle to pulse width in microseconds
    int pulseWidth = mapThrottleToPwm(throttle);

    // Update the appropriate atomic variable
    if (escPin == PIN_ESC_1) {
        esc1PulseWidth = pulseWidth;
    } else if (escPin == PIN_ESC_2) {
        esc2PulseWidth = pulseWidth;
    }

    // Debug output
    if (verboseOutput) {
        std::cout << "ESC " << escPin << ": Throttle=" << throttle
                  << ", Pulse=" << pulseWidth << "μs, Duty="
                  << (pulseWidth * 100.0 / 20000.0) << "%" << std::endl;
    }
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
        digitalWrite(PIN_LED, i % 2 ? HIGH : LOW);

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
        digitalWrite(PIN_LED, HIGH);  // LED on when battery discharged
    } else {
        if (batteryVoltage > RESET_VOLTAGE) {
            batteryDischarged = false;
        }

        // Flash LED when battery is good
        if (!batteryDischarged) {
            int currentState = digitalRead(PIN_LED);
            digitalWrite(PIN_LED, currentState ? LOW : HIGH);
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

void RobotControl::onConnectionStatedChanged(bool state)
{
}

void RobotControl::createMessage(uint8_t msgId, uint8_t rw, QByteArray payload, QByteArray *result)
{
    // Safety check
    if (!result) {
        qDebug() << "Error: Result pointer is null in createMessage";
        return;
    }

    // Clear the result array first to prevent appending to existing data
    result->clear();

    // Make sure payload size doesn't exceed buffer
    if (payload.size() > MaxPayload) {
        qDebug() << "Warning: Payload size exceeds maximum, truncating";
        payload = payload.left(MaxPayload);
    }

    // Create buffer with zero initialization
    uint8_t buffer[MaxPayload + 8] = {0};
    uint8_t command = msgId;

    // Create the packet
    int len = message.create_pack(rw, command, payload, buffer);

    // Safety check on length
    if (len <= 0 || len > (MaxPayload + 8)) {
        qDebug() << "Error: Invalid message length in createMessage:" << len;
        return;
    }

    // Copy buffer to result QByteArray
    for (int i = 0; i < len; i++) {
        result->append(static_cast<char>(buffer[i]));
    }
}

void RobotControl::requestData(uint8_t command)
{
    QByteArray payload;
    QByteArray sendData;
    createMessage(command, mRead, payload, &sendData);
    gattServer->writeValue(sendData);
}

void RobotControl::sendData(uint8_t command, uint8_t value)
{
    QByteArray payload(1, 0);  // Initialize with size 1, value 0
    // Now set the value
    payload[0] = value;
    QByteArray sendData;
    createMessage(command, mWrite, payload, &sendData);
    gattServer->writeValue(sendData);
}

void RobotControl::sendFloat(uint8_t command, float value)
{
    // Create a QByteArray to hold the float (4 bytes)
    QByteArray payload(sizeof(float), 0);

    // Convert float to bytes
    // This creates a platform-independent representation of the float
    const char* valueBytes = reinterpret_cast<const char*>(&value);

    // Copy the bytes to the payload
    for (size_t i = 0; i < sizeof(float); i++) {
        payload[i] = valueBytes[i];
    }

    // Create and send the message
    QByteArray sendData;
    createMessage(command, mWrite, payload, &sendData);
    gattServer->writeValue(sendData);
}

void RobotControl::sendString(uint8_t command, QString value)
{
    QByteArray sendData;
    QByteArray bytedata;
    bytedata = value.toLocal8Bit();
    createMessage(command, mWrite, bytedata, &sendData);
    gattServer->writeValue(sendData);
}

bool RobotControl::parseMessage(QByteArray *data, uint8_t &command, QByteArray &value, uint8_t &rw)
{
    // Safety checks
    if (!data || data->isEmpty()) {
        qDebug() << "Error: Invalid data in parseMessage";
        return false;
    }

    // Clear the output value array
    value.clear();

    // Check minimum packet size
    if (data->size() < 4) {
        qDebug() << "Error: Packet too small:" << data->size();
        return false;
    }

    // Log raw data for debugging
    QString hexDump;
    for (int i = 0; i < data->size(); i++) {
        hexDump += QString("%1 ").arg((unsigned char)data->at(i), 2, 16, QChar('0'));
    }

    // Initialize the message struct
    MessagePack parsedMessage = {0};

    // Parse the message
    uint8_t* dataToParse = reinterpret_cast<uint8_t*>(data->data());

    if (message.parse(dataToParse, static_cast<uint8_t>(data->size()), &parsedMessage)) {
        command = parsedMessage.command;
        rw = parsedMessage.rw;

        // Safety check on parsed length
        if (parsedMessage.len > MaxPayload) {
            qDebug() << "Error: Invalid parsed length:" << parsedMessage.len;
            return false;
        }

        // Copy data safely
        for (int i = 0; i < parsedMessage.len; i++) {
            value.append(static_cast<char>(parsedMessage.data[i]));
        }

        return true;
    }

    qDebug() << "Failed to parse message";
    return false;
}

void RobotControl::onDataReceived(QByteArray data)
{
    // Debug için: Alınan veriyi hexadecimal olarak göster
    QString hexDump;
    for (int i = 0; i < data.size(); i++) {
        hexDump += QString("%1 ").arg((unsigned char)data.at(i), 2, 16, QChar('0'));
    }

    try {
        // Mesajı ayrıştır
        uint8_t parsedCommand = 0;
        uint8_t rw = 0;
        QByteArray parsedValue;
        if (!parseMessage(&data, parsedCommand, parsedValue, rw)) {
            qDebug() << "Failed to parse message, skipping processing";
            return;
        }

        // İstek türüne göre işle
        if (rw == mRead) {
            // Okuma istekleri - mevcut değerleri gönder
            switch (parsedCommand) {
            case mPP:
                sendFloat(mPP, KP);
                break;
            case mPD:
                sendFloat(mPD, KD);
                break;
            case mPC:
                sendFloat(mPC, KC);
                break;
            case mPV:
                sendFloat(mPV, KV);
                break;
            case mSD:
                sendFloat(mSD, KSD);  // Burada mAC yerine mSD olmalı
                break;
            case mAC:
                sendFloat(mAC, KAC);
                break;
            case mArmed:
                sendData(mArmed, power);
                break;
            default:
                qDebug() << "Unknown command in read operation:" << parsedCommand;
                break;
            }
        } else if (rw == mWrite) {
            // Yazma istekleri - gelen değerlerle değişkenleri güncelle

            // Float parametre komutları
            if (parsedCommand == mPP || parsedCommand == mPD || parsedCommand == mPC ||
                parsedCommand == mPV || parsedCommand == mSD || parsedCommand == mAC ) {

                // Byte array'i float'a dönüştür
                if (parsedValue.size() >= sizeof(float)) {
                    float floatValue;
                    memcpy(&floatValue, parsedValue.constData(), sizeof(float));

                    // Float değeri ilgili parametreye ata
                    switch (parsedCommand) {
                    case mPP:
                        KP = floatValue;
                        qDebug() << "Updated KP to:" << KP;
                        break;
                    case mPD:
                        KD = floatValue;
                        qDebug() << "Updated KD to:" << KD;
                        break;
                    case mPC:
                        KC = floatValue;
                        qDebug() << "Updated KC to:" << KC;
                        break;
                    case mPV:
                        KV = floatValue;
                        qDebug() << "Updated KV to:" << KV;
                        break;
                    case mSD:
                        KSD = floatValue;
                        qDebug() << "Updated KSD to:" << KSD;
                        break;
                    case mAC:
                        KAC = floatValue;
                        qDebug() << "Updated KAC to:" << KAC;
                        break;
                    }
                } else {
                    qDebug() << "Error: Not enough data for float value. Expected:"
                             << sizeof(float) << "bytes, got:" << parsedValue.size() << "bytes";
                }
            }
            // Boolean değer komutu
            else if (parsedCommand == mArmed || parsedCommand == mDisArmed) {
                if (parsedCommand == mArmed) {
                    power = true;
                    powerOn();
                } else {
                    power = false;
                    powerOff();
                }
                sendData(mArmed, power);
            }
            else if (parsedCommand == mSpeak) {
                auto soundText = QString(parsedValue.data());
                qDebug() << "Speak command received:" << soundText;
            }
            // Hareket komutları
            // Hareket komutları
            else if (parsedCommand == mForward || parsedCommand == mBackward ||
                     parsedCommand == mLeft || parsedCommand == mRight) {
                // Değeri 0-100 aralığından yüzde olarak al
                int value = parsedValue.isEmpty() ? 0 : static_cast<unsigned char>(parsedValue.at(0));

                // Değeri 0-1 aralığına normalize et
                float normalizedValue = value / 100.0f;

                // Hareket parametreleri
                float forward = 0.0f;
                float lateral = 0.0f; // Balance robot'ta genellikle kullanılmaz
                float yaw = 0.0f;
                float vertical = 0.0f; // Balance robot'ta genellikle kullanılmaz
                quint16 buttons = 0;

                // Komuta göre hareket parametrelerini ayarla
                switch (parsedCommand) {
                case mForward:
                    forward = normalizedValue;
                    qDebug() << "Moving forward with value:" << value << "(" << normalizedValue << ")";
                    break;
                case mBackward:
                    forward = -normalizedValue;
                    qDebug() << "Moving backward with value:" << value << "(" << -normalizedValue << ")";
                    break;
                case mLeft:
                    yaw = -normalizedValue;
                    qDebug() << "Turning left with value:" << value << "(" << -normalizedValue << ")";
                    break;
                case mRight:
                    yaw = normalizedValue;
                    qDebug() << "Turning right with value:" << value << "(" << normalizedValue << ")";
                    break;
                }

                // Robotu hareket ettir
                bool success = moveRobot(forward, lateral, yaw, vertical, buttons);

                if (!success) {
                    qDebug() << "Failed to move robot. Power might be off or battery discharged.";
                }
            }
            // Bilinmeyen komutlar
            else {
                qDebug() << "Unknown command in write operation:" << parsedCommand;
            }
        } else {
            qDebug() << "Unknown request type (neither read nor write):" << rw;
        }
    } catch (const std::exception& e) {
        qDebug() << "Exception during message processing:" << e.what();
    } catch (...) {
        qDebug() << "Unknown exception during message processing";
    }
}

