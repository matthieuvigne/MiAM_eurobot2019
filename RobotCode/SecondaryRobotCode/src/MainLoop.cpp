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
    std::cout << "Kill called, shutting down motors" << std::endl;
    robot.stopMotors();
    std::cout << "Exiting" << std::endl;
    exit(0);
}


int main(int argc, char **argv)
{
    // Wire signals.
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    // Init beaglebone serial ports and GPIO.
    BBB_enableCape();

    // Start low-level loop.
    robot.lowLevelLoop();

    return 0;
}

