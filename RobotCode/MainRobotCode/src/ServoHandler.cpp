#include <MiAMEurobot/raspberry_pi/RPiGPIO.h>
#include "ServoHandler.h"
#include <unistd.h>

// Servo config: port defintion.
int const SERVO_SUCTION[3] = {0, 1, 2};    // Numbered from right to left.
int const SERVO_TUBE[3] = {6, 7, 8};

int const SERVO_TAP = 12;
int const SERVO_VERTICAL_TRANSLATION = 13;

// Temporary.
/// \brief Servo position definition.
//~ int const SP_SUCTION_HIGH = {1500, 1500, 1500};
//~ int const SP_SUCTION_LOW = {1500, 1500, 1500};

//~ int const SP_TUBE_OPEN = {1500, 1500, 1500};
//~ int const SP_TUBE_CLOSE = {1500, 1500, 1500};

int const SP_TAP_OPEN  = 1000;
int const SP_TAP_CLOSE  = 1000;

int const PUMP_PWM = 26;

ServoHandler::ServoHandler()
{
}


bool ServoHandler::init(std::string const& portName)
{
    // Enable pump GPIO.
    RPi_setupGPIO(PUMP_PWM, PI_GPIO_OUTPUT);
    turnOffPump();

    return maestro_.init(portName);
}


void ServoHandler::openTube(int tubeNumber)
{
    if (tubeNumber < 0 || tubeNumber > 2)
        return;
    maestro_.setPosition(SERVO_TUBE[tubeNumber], 1570);
}


void ServoHandler::closeTube(int tubeNumber)
{
	switch(tubeNumber)
	{
		case 0: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1865); break;
		case 1: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1865); break;
		case 2: maestro_.setPosition(SERVO_TUBE[tubeNumber], 1820); break;
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
		usleep(80);
	}
}

void ServoHandler::turnOnPump()
{
    RPi_writeGPIO(PUMP_PWM, HIGH);
}

void ServoHandler::turnOffPump()
{
    RPi_writeGPIO(PUMP_PWM, LOW);
}


void ServoHandler::moveSuction(bool high, bool moveMiddle)
{
    if(high)
    {
        maestro_.setPosition(SERVO_SUCTION[0], 1800);
        if (moveMiddle)
            maestro_.setPosition(SERVO_SUCTION[1], 1650);
        maestro_.setPosition(SERVO_SUCTION[2], 1800);
    }
    else
    {
        maestro_.setPosition(SERVO_SUCTION[0], 1300);
        if (moveMiddle)
            maestro_.setPosition(SERVO_SUCTION[1], 1150);
        maestro_.setPosition(SERVO_SUCTION[2], 1300);
    }
}

void ServoHandler::moveMiddleSuctionForDrop()
{
    maestro_.setPosition(SERVO_SUCTION[0], 1900);
    maestro_.setPosition(SERVO_SUCTION[1], 1750);
    maestro_.setPosition(SERVO_SUCTION[2], 1900);
}


void ServoHandler::moveRail(int velocity)
{
    maestro_.setPosition(SERVO_VERTICAL_TRANSLATION, velocity);
}
