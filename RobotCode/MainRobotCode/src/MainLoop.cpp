/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///

#include "Robot.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

Robot robot;


// Stop motor before exit.
void killCode(int x)
{
    robot.servos_.shutdownServos();
    robot.servos_.turnOffPump();
    robot.stopMotors();
    exit(0);
}


int main(int argc, char **argv)
{
    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();
    RPi_setupGPIO(21, PI_GPIO_INPUT_PULLUP); // Starting cable.


    // Init robot - this only creates the log for now.
    bool isInit = robot.initSystem();
    if (!isInit)
    {
        std::cout << "Failed to init robot" << std::endl;
        exit(-1);
    }

    // Start low-level loop.
    robot.lowLevelLoop();
    return 0;
}

