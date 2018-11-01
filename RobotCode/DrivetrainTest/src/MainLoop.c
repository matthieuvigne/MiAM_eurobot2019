/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///

#include "Robot.h"

#include <iostream>

Robot robot;

// Motor values.
const int MOTOR_KVAL_HOLD = 0x30;
const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};


// Stop motor before exit.
void killCode(int x)
{
	robot.stepperMotors_.hardStop();
	g_usleep(50000);
	robot.stepperMotors_.highZ();
	exit(0);
}

void *startLowLevelThread(void *)
{
	robot.lowLevelThread();
	return 0;
}

int main(int argc, char **argv)
{
	// Wire signals.
	signal(SIGINT, killCode);
	signal(SIGTERM, killCode);

	// Init raspberry serial ports and GPIO.
	//~ RPi_enablePorts();

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

	// Init communication with arduino.
	uCListener_startListener("/dev/arduinoUno");
	g_usleep(2000000);


	// Start low-level thread.
	g_thread_new("LowLevel", startLowLevelThread, NULL);

	// Servo along a trajectory.
	//~ RobotPosition endPosition = robot.getCurrentPosition();
	//~ endPosition.y += 500;
	//~ endPosition.x += 500;
	std::vector<std::shared_ptr<Trajectory>> traj;
	//~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), endPosition);

	std::shared_ptr<Trajectory> t = std::shared_ptr<Trajectory>(new miam::trajectory::PointTurn(robot.getCurrentPosition(), G_PI_2));
	traj.push_back(t);
	robot.setTrajectoryToFollow(traj);
	robot.waitForTrajectoryFinished();

	g_usleep(2000000);
	printf("done\n");
	while(TRUE) ;;
	return 0;
}

