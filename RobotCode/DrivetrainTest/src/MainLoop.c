/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///

#include "Robot.h"
#include "Localisation.h"

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
    L6470_hardStop(robot.motors[0]);
	L6470_highZ(robot.motors[0]);
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

	// Init raspberry serial ports and GPIO.
	RPi_enablePorts();

	L6470_initStructure(&(robot.motors[0]), RPI_SPI_00);
    L6470_initMotion(robot.motors[0], MOTOR_MAX_SPEED, MOTOR_MAX_ACCELERATION);
    L6470_initBEMF(robot.motors[0], MOTOR_KVAL_HOLD, MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
	if(L6470_getParam(robot.motors[0], dSPIN_KVAL_HOLD) != MOTOR_KVAL_HOLD)
	{
		printf("Failed to init motor\n");
		exit(0);
	}

	// Init communication with arduino.
	robot.isOnRightSide = TRUE;
	uCListener_startListener("/dev/arduinoUno");
	localisation_startSensorUpdate();

	g_usleep(2000000);
	L6470_goToPosition(robot.motors[0], 1500);
	L6470_getError(robot.motors[0]);

	PID pid = {0.0,		// integral
			   0.15,		// maxIntegral
			   200.0,		// Kp
			   0.0,		// Kd
			   0.0};		// Ki

	double dt = 0.01;

	while(TRUE)
	{
		// Get encoder position.
		robot.microcontrollerData = uCListener_getData();
		double encoderPosition = robot.microcontrollerData.encoderValues[0];
		// Get motor position.
		double motorPosition = L6470_getPosition(robot.motors[0]) * MOTOR_TICK;

		// Run PID.
		double error = encoderPosition - motorPosition;
		if(error < 0.2 && error > -0.2)
			error = 0;
		int command = PID_computeValue(&pid, error, 0.0, dt);
		//~ L6470_setSpeed(robot.motors[0], command);
		L6470_getError(robot.motors[0]);

		printf("\rEncoder: %0.2f motor: %0.2f command: %d second encoder %f", encoderPosition, motorPosition, command, robot.microcontrollerData.encoderValues[1]);
		fflush(stdout);
		g_usleep(1e6 * dt);
	}
	return 0;
}

