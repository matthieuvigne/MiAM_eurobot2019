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

    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();
    RPi_setupGPIO(21, PI_GPIO_INPUT_PULLUP); // Starting cable.


    // Init robot - this only creates the log for now.
    bool isInit = robot.initSystem();
    if (!isInit)
    {
        std::cout << "Failed to init robot" << std::endl;
        //~ exit(-1);
    }


    //~ RPi_setupGPIO(4, PI_GPIO_OUTPUT);

    //~ robot.servos_.turnOffPump();


    robot.servos_.tapClose();
    robot.servos_.closeTube(0);
    robot.servos_.closeTube(2);
    robot.servos_.openTube(1);
    //~ robot.servos_.turnOnPump();
    //~ RPi_writeGPIO(4, HIGH);
    //~ usleep(5000000);
    //~ robot.servos_.closeTube(1);
    //~ usleep(1000000);
    //~ robot.servos_.turnOffPump();
    //~ usleep(5000000);
    //~ robot.servos_.openTube(0);
    //~ robot.servos_.openTube(1);
    //~ robot.servos_.openTube(2);
    //~ robot.servos_.tapOpen();

    // Start low-level thread.
    std::thread lowLevelThread(&Robot::lowLevelThread, &robot);

    // Servo along a trajectory.
    RobotPosition endPosition = robot.getCurrentPosition();
    std::cout << "Playing first trajectory" << std::endl;
    endPosition.x += 500;
    std::vector<std::shared_ptr<Trajectory>> traj;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), endPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    //~ std::cout << "Playing second trajectory" << std::endl;
    //~ RobotPosition startPosition = endPosition;
    //~ endPosition.y += 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ std::cout << "Playing third trajectory" << std::endl;
    //~ endPosition.theta = M_PI_2;
    //~ startPosition = endPosition;
    //~ endPosition.x -= 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();
    //~ startPosition = endPosition;

    //~ std::cout << "Playing fourth trajectory" << std::endl;
    //~ endPosition.theta = M_PI;
    //~ startPosition = endPosition;
    //~ endPosition.y -= 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    usleep(200000);
    //~ robot.servos_.turnOffPump();
    printf("done\n");
    while(true) ;;
    return 0;
}

