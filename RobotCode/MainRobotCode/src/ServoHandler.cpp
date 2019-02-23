#include "ServoHandler.h"
#include <unistd.h>

// Servo config: port defintion.
int const SERVO_SUCTION[3] = {0, 1, 2};    // Numbered from right to left.
int const SERVO_TUBE[3] = {6, 7, 8};

int const SERVO_TAP = 12;
int const SERVO_VERTICAL_TRANSLATION = 13;
int const PUMP = 14;

// Temporary.
/// \brief Servo position definition.
//~ int const SP_SUCTION_HIGH = {1500, 1500, 1500};
//~ int const SP_SUCTION_LOW = {1500, 1500, 1500};

//~ int const SP_TUBE_OPEN = {1500, 1500, 1500};
//~ int const SP_TUBE_CLOSE = {1500, 1500, 1500};

int const SP_TAP_OPEN  = 1000;
int const SP_TAP_CLOSE  = 1000;

int const SP_PUMP_OFF = 1500;
int const SP_PUMP_ON = 2000;

ServoHandler::ServoHandler()
{
}


bool ServoHandler::init(std::string const& portName)
{
    return maestro_.init(portName);
}


void ServoHandler::openTube(int tubeNumber)
{
    if (tubeNumber < 0 || tubeNumber > 2)
        return;
    maestro_.setPosition(SERVO_TUBE[tubeNumber], 1550);
}


void ServoHandler::closeTube(int tubeNumber)
{
	switch(tubeNumber)
	{
		case 0: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1850); break;
		case 1: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1840); break;
		case 2: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1850); break;
		default: break;
	}
}


void ServoHandler::tapOpen()
{
    maestro_.setPosition(SERVO_TAP, 1500);
}


void ServoHandler::tapClose()
{
    maestro_.setPosition(SERVO_TAP, 750);
}

void ServoHandler::shutdownServos()
{
	for(int i = 0; i < 18; i++)
	{
		maestro_.setPosition(i, 0);
		usleep(50);
	}
}
