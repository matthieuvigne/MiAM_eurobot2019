#include "Robot.h"

// Robot dimensions, for localisation.
#define ROBOT_ENCODER_WHEEL_DIAMETER 53.0
#define ROBOT_WIDTH 300.0

#define STEP_TO_ANGLE 0.00314159

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


void *localisation_startSensorUpdate()
{
    printf("Sensor update / localisation thread started.\n");
    resetLocalisation = FALSE;

    // Create log file
    gchar *date = g_date_time_format(g_date_time_new_now_local(), "%Y%m%dT%H%M%SZ");
    gchar *filename = g_strdup_printf("log%s.csv", date);
    g_free(date);
    gchar *headers = getHeaderStringList();
    Logger logger = logger_create(filename, "Drivetrain test robot", "", headers);
    g_free(filename);
    g_free(headers);

    // Create metronome
    Metronome metronome = metronome_create(LOOP_PERIOD * 1e9);
    double currentTime = 0;
    double lastTime = 0;

    // Create kalman filter.
    Kalman kalmanFilter;
    kalman_init(&kalmanFilter, robot.currentPosition.theta);

    while(TRUE)
    {
	// Reset position, if asked for.
	if(resetLocalisation)
	{
	    resetLocalisation = FALSE;
	    if(isXReset)
		    robot.currentPosition.x = positionReset.x;
	    if(isYReset)
		    robot.currentPosition.y = positionReset.y;
	    if(isThetaReset)
	    {
		robot.currentPosition.theta = positionReset.theta;
		kalman_init(&kalmanFilter, positionReset.theta);
	    }
	}

	// Wait for next tick.
	lastTime = currentTime;
	metronome_wait(&metronome);
	currentTime = metronome_getTimeElapsed(metronome);

	// Update arduino data.
	uCData oldData = robot.microcontrollerData;
	robot.microcontrollerData = uCListener_getData();

	int motorEncoder[2];
	double motorSpeed[2];
	for(int i = 0; i < 2; i++)
	{
		motorEncoder[i]= L6470_getPosition(robot.motors[i]);
		motorSpeed[i]= L6470_getSpeed(robot.motors[i]);
	}

	double dt = currentTime - lastTime;

	// Estimate new position.

	// Estimate angle.
	double encoderIncrement[2];
	for(int i = 0; i < 2; i++)
		encoderIncrement[i] = robot.microcontrollerData.encoderValues[i] - oldData.encoderValues[i];

	double tanTheta = (encoderIncrement[RIGHT] - encoderIncrement[LEFT]) * ROBOT_ENCODER_WHEEL_DIAMETER / ROBOT_WIDTH;

	// If playing on right side, invert rotation axis to symmetrize robot motion.
	if(robot.isOnRightSide)
		tanTheta = -tanTheta;

	robot.currentPosition.theta += atan(tanTheta);
	// Integrate angle estimation on X and Y.
	double linearIncrement = 1 / 2.0 * (encoderIncrement[LEFT] + encoderIncrement[RIGHT]);
	robot.currentPosition.x += linearIncrement * cos(robot.currentPosition.theta);
	// Minus sign: the frame is indirect.
	robot.currentPosition.y -= linearIncrement * sin(robot.currentPosition.theta);

	// Log data.
	logger_setData(&logger, LOGGER_TIME, currentTime);
	logger_setData(&logger, LOGGER_COMMAND_VELOCITY_RIGHT, motorSpeed[RIGHT]);
	logger_setData(&logger, LOGGER_COMMAND_VELOCITY_LEFT, motorSpeed[LEFT]);
	logger_setData(&logger, LOGGER_MOTOR_POSITION_RIGHT, motorEncoder[RIGHT] * STEP_TO_ANGLE);
	logger_setData(&logger, LOGGER_MOTOR_POSITION_RIGHT, motorEncoder[LEFT] * STEP_TO_ANGLE);
	logger_setData(&logger, LOGGER_ENCODER_RIGHT, robot.microcontrollerData.encoderValues[RIGHT]);
	logger_setData(&logger, LOGGER_ENCODER_LEFT, robot.microcontrollerData.encoderValues[LEFT]);

	logger_setData(&logger, LOGGER_CURRENT_POSITION_X, robot.currentPosition.x);
	logger_setData(&logger, LOGGER_CURRENT_POSITION_Y, robot.currentPosition.y);
	logger_setData(&logger, LOGGER_CURRENT_POSITION_THETA, robot.currentPosition.theta);

	logger_writeLine(logger);
    }
    return 0;
}
