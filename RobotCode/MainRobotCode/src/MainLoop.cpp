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

// Motor values.
const int MOTOR_KVAL_HOLD = 0x30;
const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};


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

    // Init raspberry serial ports and GPIO.
    RPi_enablePorts();

    // Init robot - this only creates the log for now.
    robot.init();

    // Update trajectory config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeed,
                                                    robotdimensions::maxWheelAcceleration,
                                                    robotdimensions::wheelSpacing);

    // Compute max stepper motor speed.
    int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
    int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

    std::cout << "max speed:" << maxSpeed << std::endl;
    std::cout << "max acceleration:" << maxAcceleration << std::endl;

    // Initialize both motors.

    robot.stepperMotors_ = miam::L6470(RPI_SPI_00, 2);
    bool areMotorsInit = robot.stepperMotors_.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                             MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);

    if(!areMotorsInit)
    {
        printf("Failed to init stepper motors.\n");
        exit(0);
    }

    // Init communication with arduino, give it 2s to boot.
    std::thread listenerThread(uCListener_listenerThread, "/dev/ttyACM0");
    usleep(2000000);

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
    robot.waitForTrajectoryFinished();

    //~ std::vector<int> position;
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ position.push_back(500.0 / G_PI / robotdimensions::wheelRadius * 600);
    //~ std::cout << "nsteps" << 500.0 / G_PI / robotdimensions::wheelRadius * 600 << std::endl;
    //~ robot.stepperMotors_.moveNSteps(position);

    usleep(2000000);
    printf("done\n");
    while(true) ;;
    return 0;
}

