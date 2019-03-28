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
    endPosition.x += 500;
    //~ endPosition.y += 300;
    std::vector<std::shared_ptr<Trajectory>> traj;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), endPosition);

    //~ std::shared_ptr<Trajectory> t = std::shared_ptr<Trajectory>(new miam::trajectory::PointTurn(robot.getCurrentPosition(), G_PI_2));
    //~ traj.push_back(t);
    robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();
    //~ robot.servos_.turnOnPump();

    //~ std::vector<int> position;
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ std::cout << "nsteps" << 500.0 / G_PI / robotdimensions::wheelRadius * 600 << std::endl;
    //~ robot.stepperMotors_.moveNSteps(position);

    usleep(200000);
    //~ robot.servos_.turnOffPump();
    printf("done\n");
    while(true) ;;
    return 0;
}

