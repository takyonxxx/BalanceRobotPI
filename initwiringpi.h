#ifndef INITWIRINGPI_H
#define INITWIRINGPI_H

#include <wiringPi.h>
#include <iostream>
#include <cstdlib>

// Initialize wiringPi for Raspberry Pi 5
inline int initWiringPi() {
    // Initialize wiringPi in BCM GPIO numbering mode
    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Failed to initialize wiringPi! Error code." << std::endl;
        return -1;
    }

    std::cout << "wiringPi initialized successfully" << std::endl;
    return 0;
}

#endif // INITWIRINGPI_H
