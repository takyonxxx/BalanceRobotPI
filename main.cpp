#include <QCoreApplication>
#include <QCommandLineParser>
#include <QTimer>
#include <iostream>
#include <sstream>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>
#include <map>
#include <pigpio.h>  // Include pigpio library
#include <termios.h> // For terminal control
#include <unistd.h>  // For POSIX API
#include <fcntl.h>   // For file control options
#include <poll.h>    // For polling input

#include "robotcontrol.h"

// Global pointer to RobotControl for signal handling
RobotControl* g_robotControl = nullptr;
std::atomic<bool> g_running(true);

// Struct to save terminal settings
struct termios orig_termios;

// Key tracking with timestamps
struct KeyState {
    bool pressed;
    std::chrono::time_point<std::chrono::steady_clock> lastPress;
};

// Map to track key states
std::map<char, KeyState> keyStates;

// Function to reset terminal to original state
void reset_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);

    // Reset terminal color and cursor visibility
    std::cout << "\033[0m" << std::flush;     // Reset all attributes
    std::cout << "\033[?25h" << std::flush;   // Show cursor
}

// Function to set terminal to raw mode for immediate key input
void set_raw_terminal() {
    struct termios raw;

    // Save original terminal settings
    tcgetattr(STDIN_FILENO, &orig_termios);

    // Register reset function to be called at exit
    atexit(reset_terminal);

    // Set terminal to raw mode
    raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    // Apply settings
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    // Make stdin non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // Hide cursor for cleaner display
    std::cout << "\033[?25l" << std::flush;
}

// Signal handler for clean shutdown
void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;

    if (g_robotControl) {
        g_robotControl->powerOff();
    }

    g_running = false;

    // Reset terminal
    reset_terminal();

    // Terminate pigpio library
    gpioTerminate();

    QCoreApplication::quit();
}

// Clear screen and position cursor at top
void clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}

void printMenu() {
    clearScreen();
    std::cout << "\033[1;36m===== Balance Robot ESC Control =====\033[0m\n"
              << "p - Power on\n"
              << "o - Power off\n"
              << "h - Fast mode on\n"
              << "y - Slow mode\n"
              << "\033[1;33mw - Forward (press and release to stop)\n"
              << "a - Left (press and release to stop)\n"
              << "d - Right (press and release to stop)\n"
              << "s - Backward (press and release to stop)\033[0m\n"
              << "Space - Stop movement\n"
              << "\n\033[1;35m--- ESC Test Options ---\033[0m\n"
              << "1 - Test Both ESCs (alternating)\n"
              << "2 - Test Right ESC (forward)\n"
              << "3 - Test Left ESC (forward)\n"
              << "4 - Test Right ESC (reverse)\n"
              << "5 - Test Left ESC (reverse)\n"
              << "0 - Stop All ESCs\n"
              << "\n\033[1;32m--- General ---\033[0m\n"
              << "m - Show this menu\n"
              << "q - Quit\n"
              << "\033[1;36m==============================\033[0m\n";

    std::cout << "\nStatus: Ready - Press p to power on\n";
    std::cout << std::flush;
}

// Function to update status line
void updateStatus(const std::string& status) {
    // Save cursor position
    std::cout << "\033[s";

    // Move cursor to line 24, column 0
    std::cout << "\033[24;0H";

    // Clear line
    std::cout << "\033[K";

    // Print status with color
    std::cout << "Status: \033[1;32m" << status << "\033[0m";

    // Restore cursor position
    std::cout << "\033[u";

    // Flush output
    std::cout << std::flush;
}

// Debug log function that works with raw terminal mode
void debugLog(const std::string& message) {
    // Save cursor position
    std::cout << "\033[s";

    // Move to debug area (line 26)
    std::cout << "\033[26;0H";

    // Clear line
    std::cout << "\033[K";

    // Print debug message
    std::cout << "\033[1;33mDebug: " << message << "\033[0m";

    // Restore cursor position
    std::cout << "\033[u";

    // Flush output
    std::cout << std::flush;
}

// Function to update joystick values based on key states
void updateJoystickFromKeys(RobotControl* robotControl) {
    // Default to neutral position
    int16_t x_value = 0;
    int16_t y_value = 0;

    // Check if keys are pressed and adjust joystick values
    if (keyStates['w'].pressed) y_value += 70;   // Forward - higher value
    if (keyStates['s'].pressed) y_value -= 70;   // Backward - higher value
    if (keyStates['a'].pressed) x_value -= 70;   // Left - higher value
    if (keyStates['d'].pressed) x_value += 70;   // Right - higher value

    // Apply the calculated joystick values
    robotControl->setJoystickInput(x_value, y_value);

    // Debug output for joystick values
    std::stringstream ss;
    ss << "Joystick values: X=" << x_value << ", Y=" << y_value;
    debugLog(ss.str());
}

// Function to handle real-time keyboard controls
void keyboardThread() {
    // Set terminal to raw mode for immediate key input
    set_raw_terminal();

    // Print the menu
    printMenu();

    // Initialize key states for WASD
    keyStates['w'] = {false, std::chrono::steady_clock::now()};
    keyStates['a'] = {false, std::chrono::steady_clock::now()};
    keyStates['s'] = {false, std::chrono::steady_clock::now()};
    keyStates['d'] = {false, std::chrono::steady_clock::now()};

    // Polling structure for keyboard input
    struct pollfd fds[1];
    fds[0].fd = STDIN_FILENO;
    fds[0].events = POLLIN;

    // For debugging key presses
    updateStatus("Ready - Press p to power on. Press and release WASD for movement.");

    // Define key timeout (how long a key is considered "pressed" after last detection)
    const auto KEY_TIMEOUT = std::chrono::milliseconds(80); // Shorter timeout

    char ch;
    while (g_running) {
        // Get current time for key timeout checking
        auto currentTime = std::chrono::steady_clock::now();

        // Check for keyboard input with a short timeout
        int poll_result = poll(fds, 1, 50); // 50ms timeout

        if (poll_result > 0 && (fds[0].revents & POLLIN)) {
            // Read the character
            if (read(STDIN_FILENO, &ch, 1) > 0) {
                // Debug: Show the received character
                std::stringstream ss;
                ss << "Key pressed: '" << ch << "' (ASCII: " << (int)ch << ")";
                debugLog(ss.str());

                if (g_robotControl) {
                    switch (ch) {
                    case 'p': // Power on
                        g_robotControl->powerOn();
                        updateStatus("Powered ON");
                        break;
                    case 'o': // Power off
                        g_robotControl->powerOff();
                        keyStates['w'].pressed = false;
                        keyStates['a'].pressed = false;
                        keyStates['s'].pressed = false;
                        keyStates['d'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        updateStatus("Powered OFF");
                        break;

                    case 'h': // Fast mode on
                        g_robotControl->setFastMode(true);
                        updateStatus("Fast mode ON");
                        break;
                    case 'y': // Slow mode (fast mode off)
                        g_robotControl->setFastMode(false);
                        updateStatus("Slow mode ON");
                        break;

                        // Movement keys - key down events
                    case 'w': // Forward
                        keyStates['w'].pressed = true;
                        keyStates['w'].lastPress = currentTime;
                        // Make sure other opposite directional key is released
                        keyStates['s'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        // Apply immediately with stronger force for instant response
                        g_robotControl->setJoystickInput(0, 80); // Stronger initial push
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        updateJoystickFromKeys(g_robotControl); // Return to normal value
                        updateStatus("Moving FORWARD");
                        break;
                    case 'a': // Left
                        keyStates['a'].pressed = true;
                        keyStates['a'].lastPress = currentTime;
                        // Make sure other opposite directional key is released
                        keyStates['d'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        // Apply immediately with stronger force for instant response
                        g_robotControl->setJoystickInput(-80, 0); // Stronger initial push
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        updateJoystickFromKeys(g_robotControl); // Return to normal value
                        updateStatus("Turning LEFT");
                        break;
                    case 'd': // Right
                        keyStates['d'].pressed = true;
                        keyStates['d'].lastPress = currentTime;
                        // Make sure other opposite directional key is released
                        keyStates['a'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        // Apply immediately with stronger force for instant response
                        g_robotControl->setJoystickInput(80, 0); // Stronger initial push
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        updateJoystickFromKeys(g_robotControl); // Return to normal value
                        updateStatus("Turning RIGHT");
                        break;
                    case 's': // Backward
                        keyStates['s'].pressed = true;
                        keyStates['s'].lastPress = currentTime;
                        // Make sure other opposite directional key is released
                        keyStates['w'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        // Apply immediately with stronger force for instant response
                        g_robotControl->setJoystickInput(0, -80); // Stronger initial push
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        updateJoystickFromKeys(g_robotControl); // Return to normal value
                        updateStatus("Moving BACKWARD");
                        break;

                    case ' ': // Space - stop all movement
                        keyStates['w'].pressed = false;
                        keyStates['a'].pressed = false;
                        keyStates['s'].pressed = false;
                        keyStates['d'].pressed = false;
                        updateJoystickFromKeys(g_robotControl);
                        updateStatus("STOPPED");
                        break;

                        // ESC test options
                    case '1': // Test All ESCs
                        g_robotControl->testAllEscs(10); // 10 seconds
                        updateStatus("ESC test complete");
                        break;
                    case '2': // Test Right ESC
                        g_robotControl->setEscPwm(50, RobotControl::PIN_ESC_1);
                        updateStatus("Testing Right ESC at 50% forward");
                        break;
                    case '3': // Test Left ESC
                        g_robotControl->setEscPwm(50, RobotControl::PIN_ESC_2);
                        updateStatus("Testing Left ESC at 50% forward");
                        break;
                    case '4': // Test Right ESC Reverse
                        g_robotControl->setEscPwm(-50, RobotControl::PIN_ESC_1);
                        updateStatus("Testing Right ESC at 50% reverse");
                        break;
                    case '5': // Test Left ESC Reverse
                        g_robotControl->setEscPwm(-50, RobotControl::PIN_ESC_2);
                        updateStatus("Testing Left ESC at 50% reverse");
                        break;
                    case '0': // Stop All ESCs
                        g_robotControl->stopAllEscs();
                        updateStatus("ESCs stopped");
                        break;

                    case 'm': // Print menu
                        printMenu();
                        break;
                    case 'q': // Quit
                        updateStatus("Quitting...");
                        g_running = false;
                        reset_terminal();
                        QCoreApplication::quit();
                        return;
                    default:
                        // Check for Escape key and other special keys
                        if (ch == 27) {
                            // Read additional bytes for escape sequences
                            char seq[2];
                            if (read(STDIN_FILENO, &seq[0], 1) <= 0) break;
                            if (read(STDIN_FILENO, &seq[1], 1) <= 0) break;

                            // Debug escape sequence
                            std::stringstream ss;
                            ss << "Escape sequence: " << (int)seq[0] << " " << (int)seq[1];
                            debugLog(ss.str());

                            // Process arrow keys as simulated key presses
                            if (seq[0] == '[') {
                                switch (seq[1]) {
                                case 'A': // Up arrow - forward
                                    keyStates['w'].pressed = true;
                                    keyStates['w'].lastPress = currentTime;
                                    updateJoystickFromKeys(g_robotControl);
                                    updateStatus("Moving FORWARD (Up arrow)");
                                    break;
                                case 'B': // Down arrow - backward
                                    keyStates['s'].pressed = true;
                                    keyStates['s'].lastPress = currentTime;
                                    updateJoystickFromKeys(g_robotControl);
                                    updateStatus("Moving BACKWARD (Down arrow)");
                                    break;
                                case 'C': // Right arrow - right
                                    keyStates['d'].pressed = true;
                                    keyStates['d'].lastPress = currentTime;
                                    updateJoystickFromKeys(g_robotControl);
                                    updateStatus("Turning RIGHT (Right arrow)");
                                    break;
                                case 'D': // Left arrow - left
                                    keyStates['a'].pressed = true;
                                    keyStates['a'].lastPress = currentTime;
                                    updateJoystickFromKeys(g_robotControl);
                                    updateStatus("Turning LEFT (Left arrow)");
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }

        // Check for key timeouts (key release detection)
        bool keyStateChanged = false;
        for (auto& key : {'w', 'a', 's', 'd'}) {
            // If key was pressed and hasn't been seen recently, consider it released
            if (keyStates[key].pressed) {
                auto elapsedTime = currentTime - keyStates[key].lastPress;
                if (elapsedTime > KEY_TIMEOUT) {
                    keyStates[key].pressed = false;
                    keyStateChanged = true;
                    std::stringstream ss;
                    ss << "Key '" << key << "' released (timeout)";
                    debugLog(ss.str());
                }
            }
        }

        // If any key state changed, update joystick values
        if (keyStateChanged) {
            updateJoystickFromKeys(g_robotControl);
            // If all keys are released, update status
            if (!keyStates['w'].pressed && !keyStates['a'].pressed &&
                !keyStates['s'].pressed && !keyStates['d'].pressed) {
                updateStatus("Stopped (keys released)");
            }
        }

        // Small delay to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char *argv[])
{
    // Setup signal handling for Ctrl+C
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Create Qt application
    QCoreApplication app(argc, argv);
    app.setApplicationName("ESC Balance Robot");
    app.setApplicationVersion("1.0");

    // Command line parser
    QCommandLineParser parser;
    parser.setApplicationDescription("Balancing Robot with ESC Control Application");
    parser.addHelpOption();
    parser.addVersionOption();

    // Add command line options
    QCommandLineOption fastModeOption(QStringList() << "f" << "fast", "Start in fast mode");
    parser.addOption(fastModeOption);

    QCommandLineOption autoPowerOption(QStringList() << "p" << "power-on", "Automatically power on after startup");
    parser.addOption(autoPowerOption);

    QCommandLineOption debugOption(QStringList() << "d" << "debug", "Enable debug output");
    parser.addOption(debugOption);

    // Process command line arguments
    parser.process(app);

    // Initialize pigpio before creating the RobotControl instance
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio! Exiting." << std::endl;
        return 1;
    }
    std::cout << "pigpio initialized successfully" << std::endl;

    // Create the robot control instance
    RobotControl robotControl;
    g_robotControl = &robotControl;

    // Connect to telemetry signal for logging
    bool debugMode = parser.isSet(debugOption);
    QObject::connect(&robotControl, &RobotControl::telemetryData,
                     [debugMode](float batteryVoltage, float loopTime) {
                         if (debugMode) {
                             // Create a string with the data
                             std::stringstream ss;
                             ss << "Battery: " << batteryVoltage << "V, Loop: "
                                << loopTime << "ms";
                             debugLog(ss.str());
                         }
                     });

    // Connect to battery low signal
    QObject::connect(&robotControl, &RobotControl::batteryLow,
                     []() {
                         debugLog("WARNING: Battery low!");
                     });

    // Connect to loop overrun signal
    QObject::connect(&robotControl, &RobotControl::loopOverrun,
                     [](float actualTime) {
                         std::stringstream ss;
                         ss << "WARNING: Loop time exceeded! Actual time: "
                            << actualTime << "ms";
                         debugLog(ss.str());
                     });

    // Initialize robot control
    if (!robotControl.initialize()) {
        std::cerr << "Failed to initialize robot control! Exiting." << std::endl;
        gpioTerminate();
        return 1;
    }

    // Apply command line options
    if (parser.isSet(fastModeOption)) {
        updateStatus("Fast mode enabled from command line");
        robotControl.setFastMode(true);
    }

    if (parser.isSet(autoPowerOption)) {
        updateStatus("Auto-powering on in 3 seconds...");
        QTimer::singleShot(3000, &robotControl, &RobotControl::powerOn);
    }

    // Start keyboard input thread
    std::thread inputThread(keyboardThread);
    inputThread.detach(); // Detach thread to allow it to run independently

    // Run the application
    int result = app.exec();

    // Set running flag to false to stop input thread
    g_running = false;

    // Give the input thread time to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Make sure we reset the terminal
    reset_terminal();

    // Terminate pigpio before exiting
    gpioTerminate();

    return result;
}
