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
    robot.stopMotors();
    exit(0);
}


int main(int argc, char **argv)
{
    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Init beaglebone serial ports and GPIO.
    BBB_enableCape();

    // Init robot - this only creates the log for now.
    bool isInit = robot.initSystem();
    if (!isInit)
    {
        std::cout << "Failed to init robot" << std::endl;
        exit(-1);
    }
    // Move servos to their initial position.
    robot.moveServos(false);

    // Lower servos.
    usleep(1000000);
    robot.moveServos(true);

    // Start low-level loop.
    robot.lowLevelLoop();

    return 0;
}

