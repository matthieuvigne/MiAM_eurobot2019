#include "Robot.h"

// Define robot constants.
const double STEP_TO_SI = G_PI * 90.0 / 400.0;

const double ROBOT_WIDTH = 340;
const double CHASSIS_BACK = 145;
const double CHASSIS_FRONT = 145;
const double CHASSIS_SIDE = 155;
const double CANON_OFFSET = 103;
const double CLAW_OFFSET = 120;
const double MOUSE_SENSOR_OFFSET = 110;
const double BALL_LENGTH_OFFSET = 156;
const double BALL_WIDTH_OFFSET = 29;
const double BIN_OFFSET = 105;

const double GYRO_Z_BIAS = -0.00002;

const int RIGHT = 1;
const int LEFT = 0;

RobotPosition startingPosition = {0,0,0};

// Implementation of thread-safe robotPosition position and robotPosition target.
RobotPosition robotPosition;
GMutex positionMutex;

double robot_getPositionX()
{
	g_mutex_lock(&positionMutex);
	double x = robotPosition.x;
	g_mutex_unlock(&positionMutex);
	return x;
}

double robot_getPositionY()
{
	g_mutex_lock(&positionMutex);
	double Y = robotPosition.y;
	g_mutex_unlock(&positionMutex);
	return Y;
}

double robot_getPositionTheta()
{
	g_mutex_lock(&positionMutex);
	double t = robotPosition.theta;
	g_mutex_unlock(&positionMutex);
	return t;
}

void robot_setPositionX(double x)
{
	g_mutex_lock(&positionMutex);
	robotPosition.x = x;
	g_mutex_unlock(&positionMutex);
}

void robot_setPositionY(double y)
{
	g_mutex_lock(&positionMutex);
	robotPosition.y = y;
	g_mutex_unlock(&positionMutex);
}

void robot_setPositionTheta(double t)
{
	g_mutex_lock(&positionMutex);
	robotPosition.theta = t;
	g_mutex_unlock(&positionMutex);
}

void robot_setPosition(RobotPosition pos)
{
	g_mutex_lock(&positionMutex);
	robotPosition.x = pos.x;
	robotPosition.y = pos.y;
	robotPosition.theta = pos.theta;
	g_mutex_unlock(&positionMutex);
}

RobotPosition robot_getPosition()
{
	RobotPosition pos;
	g_mutex_lock(&positionMutex);
	pos.x = robotPosition.x;
	pos.y = robotPosition.y;
	pos.theta = robotPosition.theta;
	g_mutex_unlock(&positionMutex);
	return pos;
}

// Create header string.
#define GENERATE_STRING(STRING) #STRING,
static const char *LOGGER_HEADERS[] = {
    LOGGER_VALUES(GENERATE_STRING)
    NULL
};


gchar *getHeaderStringList()
{
	// "logger_" prefix to remove.
	int const PREFIX_LENGTH = 7;
	// Put first element in headerString.
	gchar *lowercase = g_ascii_strdown(LOGGER_HEADERS[0], -1);
	gchar *headerString = g_strdup(&lowercase[PREFIX_LENGTH]);
	g_free(lowercase);
	int i = 1;
	while(LOGGER_HEADERS[i] != NULL)
	{
		// Put string in lowercase, remove prefix and append.
		gchar *lowercase = g_ascii_strdown(LOGGER_HEADERS[i], -1);
		gchar *concatenated = g_strjoin(",", headerString, &lowercase[PREFIX_LENGTH], NULL);
		g_free(lowercase);
		g_free(headerString);
		headerString = concatenated;
		i++;
	}
	return headerString;
}



// Check IR sensors for an obstacle, updating global variables.

gboolean robot_disableIRWater = FALSE;
gboolean robot_disableIROnStartup = FALSE;

// Sensor value threshold to consider a valid detection.
const int IR_FRONT_THRESHOLD = 850;
const int IR_BACK_THRESHOLD = 850;

# define AVR_WIN 10
int frontIR[2][AVR_WIN];
int backIR[2][AVR_WIN];

void robot_checkIRSensors(Logger *logger)
{
	if(robot_disableIROnStartup)
	{
		robot_IRDetectionBack = FALSE;
		robot_IRDetectionFront = FALSE;
		return;
	}
	// Get IR values, filter them using a moving average, then update variable and led status accordingly.
	int backRight = gpio_analogRead(CAPE_ANALOG[3]);
	int backLeft = gpio_analogRead(CAPE_ANALOG[4]);

	for(int i = 1; i < AVR_WIN; i++)
	{
		backIR[0][i-1] = backIR[0][i];
		backIR[1][i-1] = backIR[1][i];
	}
	backIR[0][AVR_WIN-1] = backRight;
	backIR[1][AVR_WIN-1] = backLeft;
	double backRightAverage = 0, backLeftAverage = 0;
	for(int i = 0; i < AVR_WIN; i++)
	{
		backRightAverage += backIR[0][i];
		backLeftAverage += backIR[1][i];
	}
	backRightAverage /= (float)(AVR_WIN);
	backLeftAverage /= (float)(AVR_WIN);

	gboolean sensorValue = backLeftAverage > IR_BACK_THRESHOLD;
	// Disable this IR sensor given that the water-opening arm is in front of it in this position.
	if(!robot_disableIRWater)
		sensorValue |= backRightAverage > IR_BACK_THRESHOLD;

	robot_IRDetectionBack = sensorValue;

	int frontRight = gpio_analogRead(CAPE_ANALOG[2]);
	int frontLeft = gpio_analogRead(CAPE_ANALOG[5]) * 1.1;


	for(int i = 1; i < AVR_WIN; i++)
	{
		frontIR[0][i-1] = frontIR[0][i];
		frontIR[1][i-1] = frontIR[1][i];
	}
	frontIR[0][AVR_WIN-1] = frontRight;
	frontIR[1][AVR_WIN-1] = frontLeft;
	double frontRightAverage = 0, frontLeftAverage = 0;
	for(int i = 0; i < AVR_WIN; i++)
	{
		frontRightAverage += frontIR[0][i];
		frontLeftAverage += frontIR[1][i];
	}
	frontRightAverage /= (float)(AVR_WIN);
	frontLeftAverage /= (float)(AVR_WIN);

	sensorValue = (frontRightAverage > IR_FRONT_THRESHOLD) ||
	              (frontLeftAverage > IR_FRONT_THRESHOLD);

	robot_IRDetectionFront = sensorValue;
	// Turn on red led if we see something on any sensor.
	if(robot_IRDetectionBack || robot_IRDetectionFront)
		gpio_digitalWrite(CAPE_LED[1], 1);
	else
		gpio_digitalWrite(CAPE_LED[1], 0);

	// Log results.
	logger_setData(logger, LOGGER_IR_FRONT_RIGHT, frontRight);
	logger_setData(logger, LOGGER_IR_FRONT_LEFT, frontLeft);
	logger_setData(logger, LOGGER_IR_BACK_RIGHT, backRight);
	logger_setData(logger, LOGGER_IR_BACK_LEFT, backLeft);
	logger_setData(logger, LOGGER_IR_FRONT_DETECT, robot_IRDetectionFront);
	logger_setData(logger, LOGGER_IR_BACK_DETECT, robot_IRDetectionBack);

	return;
}
