#include <QCoreApplication>
#include <QCommandLineParser>
#include <QTimer>
#include <iostream>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>

#include "robotcontrol.h"

// Global pointer to RobotControl for signal handling
RobotControl* g_robotControl = nullptr;
std::atomic<bool> g_running(true);

// Signal handler for clean shutdown
void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;

    if (g_robotControl) {
        g_robotControl->powerOff();
    }

    g_running = false;
    QCoreApplication::quit();
}

void printMenu() {
    std::cout << "\n===== Balance Robot ESC Control =====\n"
              << "p - Power on\n"
              << "o - Power off\n"
              << "h - Fast mode on\n"
              << "y - Slow mode\n"
              << "w - Forward\n"
              << "a - Left\n"
              << "d - Right\n"
              << "s - Backward\n"
              << "Space - Stop movement\n"
              << "\n--- ESC Test Options ---\n"
              << "1 - Test Both ESCs (alternating)\n"
              << "2 - Test Right ESC (forward)\n"
              << "3 - Test Left ESC (forward)\n"
              << "4 - Test Right ESC (reverse)\n"
              << "5 - Test Left ESC (reverse)\n"
              << "0 - Stop All ESCs\n"
              << "\n--- General ---\n"
              << "m - Show this menu\n"
              << "q - Quit\n"
              << "==============================\n";
}

// Function to handle keyboard input in a separate thread
void keyboardInputThread() {
    char cmd;
    while (g_running) {
        // Get a command from the user
        std::cin >> cmd;

        if (g_robotControl) {
            switch (cmd) {
            case 'p': // Power on
                g_robotControl->powerOn();
                std::cout << "\nPower ON" << std::endl;
                break;
            case 'o': // Power off
                g_robotControl->powerOff();
                std::cout << "\nPower OFF" << std::endl;
                break;
            case 'h': // Fast mode on
                g_robotControl->setFastMode(true);
                std::cout << "\nFast mode ON" << std::endl;
                break;
            case 'y': // Slow mode (fast mode off)
                g_robotControl->setFastMode(false);
                std::cout << "\nSlow mode ON" << std::endl;
                break;
            case 'w': // Forward
                g_robotControl->setJoystickInput(0, 50);
                std::cout << "\nMoving FORWARD" << std::endl;
                break;
            case 'a': // Left
                g_robotControl->setJoystickInput(-50, 0);
                std::cout << "\nTurning LEFT" << std::endl;
                break;
            case 'd': // Right
                g_robotControl->setJoystickInput(50, 0);
                std::cout << "\nTurning RIGHT" << std::endl;
                break;
            case 's': // Backward
                g_robotControl->setJoystickInput(0, -50);
                std::cout << "\nMoving BACKWARD" << std::endl;
                break;
            case ' ': // Stop
                g_robotControl->setJoystickInput(0, 0);
                std::cout << "\nSTOPPED" << std::endl;
                break;

                // ESC test options
            case '1': // Test All ESCs
                g_robotControl->testAllEscs(10); // 10 seconds
                break;
            case '2': // Test Right ESC
                g_robotControl->setEscPwm(50, RobotControl::PIN_ESC_1);
                std::cout << "\nTesting Right ESC at 50% forward" << std::endl;
                break;
            case '3': // Test Left ESC
                g_robotControl->setEscPwm(50, RobotControl::PIN_ESC_2);
                std::cout << "\nTesting Left ESC at 50% forward" << std::endl;
                break;
            case '4': // Test Right ESC Reverse
                g_robotControl->setEscPwm(-50, RobotControl::PIN_ESC_1);
                std::cout << "\nTesting Right ESC at 50% reverse" << std::endl;
                break;
            case '5': // Test Left ESC Reverse
                g_robotControl->setEscPwm(-50, RobotControl::PIN_ESC_2);
                std::cout << "\nTesting Left ESC at 50% reverse" << std::endl;
                break;
            case '0': // Stop All ESCs
                g_robotControl->stopAllEscs();
                break;

            case 'm': // Print menu
                printMenu();
                break;
            case 'q': // Quit
                std::cout << "\nQuitting..." << std::endl;
                g_running = false;
                QCoreApplication::quit();
                break;
            default:
                // Ignore other keys
                break;
            }
        }
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

    // Create the robot control instance
    RobotControl robotControl;
    g_robotControl = &robotControl;

    // Connect to telemetry signal for logging
    bool debugMode = parser.isSet(debugOption);
    QObject::connect(&robotControl, &RobotControl::telemetryData,
                     [debugMode](float batteryVoltage, float loopTime) {
                         if (debugMode) {
                             std::cout << "Battery: " << batteryVoltage << "V, Loop: "
                                       << loopTime << "ms   \r" << std::flush;
                         }
                     });

    // Connect to battery low signal
    QObject::connect(&robotControl, &RobotControl::batteryLow,
                     []() {
                         std::cout << "\nWARNING: Battery low!" << std::endl;
                     });

    // Connect to loop overrun signal
    QObject::connect(&robotControl, &RobotControl::loopOverrun,
                     [](float actualTime) {
                         std::cout << "\nWARNING: Loop time exceeded! Actual time: "
                                   << actualTime << "ms" << std::endl;
                     });

    // Initialize robot control
    if (!robotControl.initialize()) {
        std::cerr << "Failed to initialize robot control! Exiting." << std::endl;
        return 1;
    }

    // Apply command line options
    if (parser.isSet(fastModeOption)) {
        std::cout << "Fast mode enabled from command line" << std::endl;
        robotControl.setFastMode(true);
    }

    if (parser.isSet(autoPowerOption)) {
        std::cout << "Auto-powering on in 3 seconds..." << std::endl;
        QTimer::singleShot(3000, &robotControl, &RobotControl::powerOn);
    }

    // Print initial menu
    printMenu();

    // Start keyboard input thread
    std::thread inputThread(keyboardInputThread);
    inputThread.detach(); // Detach thread to allow it to run independently

    // Run the application
    int result = app.exec();

    // Set running flag to false to stop input thread
    g_running = false;

    // Give the input thread time to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return result;
}
