/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///

#include "Robot.h"

Robot robot;

// Motor values.
const int MOTOR_KVAL_HOLD = 0x20;
const int MOTOR_BEMF[4] = {0x30, 0x172E, 0xC, 0x20};
const int MOTOR_MAX_SPEED = 1500;
const int MOTOR_MAX_ACCELERATION = 5000;

const double MOTOR_TICK = 2 * G_PI / 200.0;

// Stop motor before exit.
void killCode()
{
    L6470_hardStop(robot.motor);
	L6470_highZ(robot.motor);
	exit(0);
}

// Generic implementation of a PID controller.
typedef struct{
	double integral; 	///< Integral value.
	double maxIntegral;	///< Maximum value of the integral feedback, i.e Ki * integral (to prevent windup).
	double Kp;			///< Proportional gain.
	double Kd;			///< Derivative gain.
	double Ki;			///< Integral gain.
}PID;

gboolean stopOnSwitch = FALSE;

// Compute next value of PID command, given the new error and the elapsed time.
double PID_computeValue(PID *pid, double positionError, double velocityError, double dt)
{
	// Compute and saturate integral.
	pid->integral += positionError * dt;
	// Prevent division by 0.
	if(ABS(pid->Ki) > 1e-6)
	{
		if(pid->Ki * pid->integral > pid->maxIntegral)
			pid->integral = pid->maxIntegral / pid->Ki;
		if(pid->Ki * pid->integral < -pid->maxIntegral)
			pid->integral = -pid->maxIntegral / pid->Ki;
	}
	// Output PID command.
	return pid->Kp * (positionError + pid->Kd * velocityError + pid->Ki * pid->integral);
}


int main(int argc, char **argv)
{
	signal(SIGINT, killCode);
	signal(SIGTERM, killCode);

	// Init raspberry serial ports.
	RPi_enablePorts();

	// Init motor.
	L6470_initStructure(&(robot.motor), RPI_SPI_00);
    L6470_initMotion(robot.motor, MOTOR_MAX_SPEED, MOTOR_MAX_ACCELERATION);
    L6470_initBEMF(robot.motor, MOTOR_KVAL_HOLD, MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
	if(L6470_getParam(robot.motor, dSPIN_KVAL_HOLD) != MOTOR_KVAL_HOLD)
	{
		printf("Failed to init motor\n");
		exit(0);
	}

	//~ L6470_goToPosition(robot.motor, 500);
	L6470_getError(robot.motor);

	// Init communication with arduino.
	uCListener_startListener("/dev/ttyACM0");

	PID pid = {0.0,		// integral
			   0.15,		// maxIntegral
			   800.0,		// Kp
			   0.0,		// Kd
			   0.0};		// Ki

	double dt = 0.01;

	while(TRUE)
	{
		// Get encoder position.
		robot.microcontrollerData = uCListener_getData();
		double encoderPosition = robot.microcontrollerData.encoderValue;
		// Get motor position.
		double motorPosition = L6470_getPosition(robot.motor) * MOTOR_TICK;

		// Run PID.
		double error = encoderPosition - motorPosition;
		if(error < 0.2 && error > -0.2)
			error = 0;
		int command = PID_computeValue(&pid, error, 0.0, dt);
		L6470_setSpeed(robot.motor, command);
		L6470_getError(robot.motor);

		printf("\rEncoder: %0.2f motor: %0.2f command: %d", encoderPosition, motorPosition, command);
		fflush(stdout);
		g_usleep(1e6 * dt);
	}
	return 0;
}

