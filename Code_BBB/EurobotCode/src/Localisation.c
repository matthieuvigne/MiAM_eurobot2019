#include "Robot.h"
#include "MotionController.h"

// Update loop frequency
const double LOOP_PERIOD = 0.010;
gboolean resetLocalisation;
RobotPosition positionReset;
gboolean isXReset, isYReset, isThetaReset;

void localisation_reset(RobotPosition resetPosition, gboolean resetX, gboolean resetY, gboolean resetTheta)
{
	positionReset = resetPosition;
	isXReset = resetX;
	isYReset = resetY;
	isThetaReset = resetTheta;
	resetLocalisation = TRUE;
	g_usleep(1.1 * 1000000 * LOOP_PERIOD);
}


void *localisation_start()
{
	printf("Localisation thread started.\n");
	resetLocalisation = FALSE;

	// Create log file
	gchar *date = g_date_time_format(g_date_time_new_now_local(), "%Y%m%dT%H%M%SZ");
	gchar *filename = g_strdup_printf("log%s.csv", date);
	g_free(date);
	gchar *headers = getHeaderStringList();
	Logger logger = logger_create(filename, "MagnetometerTest", "", headers);
	g_free(filename);
	g_free(headers);

	// Create metronome
	Metronome metronome = metronome_create(LOOP_PERIOD * 1e9);
	double currentTime = 0;
	double lastTime = 0;

	// Create kalman filter.
	Kalman kalmanFilter;
	kalman_init(&kalmanFilter, robot_getPositionTheta());

	int oldEncoder[2] = {0,0};

	double intMouseX = 0;
	double intMouseY = 0;

	while(TRUE)
	{
		// Reset position, if asked for.
		if(resetLocalisation)
		{
			resetLocalisation = FALSE;
			if(isXReset)
				robot_setPositionX(positionReset.x);
			if(isYReset)
				robot_setPositionY(positionReset.y);
			if(isThetaReset)
			{
				robot_setPositionTheta(positionReset.theta);
				kalman_init(&kalmanFilter, positionReset.theta);
			}
		}

		// Wait for next tick.
		lastTime = currentTime;
		metronome_wait(&metronome);
		currentTime = metronome_getTimeElapsed(metronome);

		// Get sensor data
		int encoder[2];
		double motorSpeed[2];
		double motorIncrementSI[2];
		for(int i = 0; i < 2; i++)
		{
			encoder[i]= L6470_getPosition(robotMotors[i]);
			motorIncrementSI[i] = (encoder[i] - oldEncoder[i]) * STEP_TO_SI;
			oldEncoder[i] = encoder[i];
			//~ motorSpeed[i]= L6470_getSpeed(robotMotors[i]);
		}

		double gyroZ = imu_gyroGetZAxis(robotIMU);
		gyroZ -= GYRO_Z_BIAS;

		double magnetoX = imu_magnetoGetXAxis(robotIMU);
		double magnetoY = imu_magnetoGetYAxis(robotIMU);


		double mouseX = 0, mouseY = 0;
		ADNS9800_getMotion(robotMouseSensor, &mouseX, &mouseY);

		intMouseX += mouseX;
		intMouseY += mouseY;

		double dt = currentTime - lastTime;
		// Estimate new position.
		RobotPosition currentPosition = robot_getPosition();
		// Estimate angle.
		double tanTheta = (motorIncrementSI[RIGHT] - motorIncrementSI[LEFT]) / ROBOT_WIDTH;
		// If playing on right side, invert rotation axis to symmetrize robot motion.
		if(robot_isOnRightSide)
		{
			tanTheta = -tanTheta;
			gyroZ = -gyroZ;
		}
		double newAngleEncoder = robot_getPositionTheta() + atan(tanTheta);
		currentPosition.theta = kalman_updateEstimate(&kalmanFilter, newAngleEncoder, gyroZ, dt);

		// Integrate angle estimation on X and Y.
		double linearIncrement = 1 / 2.0 * (motorIncrementSI[LEFT] + motorIncrementSI[RIGHT]);
		currentPosition.x += linearIncrement * cos(currentPosition.theta);
		// Minus sign: the frame is indirect.
		currentPosition.y -= linearIncrement * sin(currentPosition.theta);
		robot_setPosition(currentPosition);

		// Log data.
		logger_setData(&logger, LOGGER_TIME, currentTime);
		logger_setData(&logger, LOGGER_COMMAND_VELOCITY_RIGHT, motorSpeed[RIGHT]);
		logger_setData(&logger, LOGGER_COMMAND_VELOCITY_LEFT, motorSpeed[LEFT]);
		logger_setData(&logger, LOGGER_ENCODER_RIGHT, encoder[RIGHT]);
		logger_setData(&logger, LOGGER_ENCODER_LEFT, encoder[LEFT]);
		logger_setData(&logger, LOGGER_GYRO_Z, gyroZ);
		logger_setData(&logger, LOGGER_ESTIMATED_BIAS, kalmanFilter.bias);
		logger_setData(&logger, LOGGER_MOUSE_X, mouseX);
		logger_setData(&logger, LOGGER_MOUSE_Y, mouseY);
		logger_setData(&logger, LOGGER_CURRENT_POSITION_X, currentPosition.x);
		logger_setData(&logger, LOGGER_CURRENT_POSITION_Y, currentPosition.y);
		logger_setData(&logger, LOGGER_CURRENT_POSITION_THETA, currentPosition.theta);
		logger_setData(&logger, LOGGER_MAGNETO_X, magnetoX);
		logger_setData(&logger, LOGGER_MAGNETO_Y, magnetoY);
		logger_setData(&logger, LOGGER_INTEGRAL_MOUSE_X, intMouseX);
		logger_setData(&logger, LOGGER_INTEGRAL_MOUSE_Y, intMouseY);
		logger_writeLine(logger);
	}
	return 0;
}
