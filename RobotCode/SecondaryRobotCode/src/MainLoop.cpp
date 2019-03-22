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
    robot.stepperMotors_.hardStop();
    usleep(50000);
    robot.stepperMotors_.highZ();
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

    // Start low-level thread.
    std::thread lowLevelThread(&Robot::lowLevelThread, &robot);

    //~ std::vector<int> position;
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ std::cout << "nsteps" << 500.0 / G_PI / robotdimensions::wheelRadius * 600 << std::endl;
    //~ robot.stepperMotors_.moveNSteps(position);

    printf("done\n");
    while(true) ;;
    return 0;
}

