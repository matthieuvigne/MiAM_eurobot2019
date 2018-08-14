/// \file Robot.h
/// \brief This file defines global variables representing the hardware on the robot, that needs sharing between
///        different files.
///
/// \details This header is included in all other source files. It contains variables representing each element of the
///			 robot, as well as global variable. The current position of the robot, and its target, are made available
///			 through mutex-proetected variables. For all other elements of this file, it is to the user to ensure
///			 no race condition may occur. Note that the I2C port is implemented in a thread-safe way, so as long
///			 as the object themselves are not modified, there should be no issue.
#ifndef ROBOT_H
     #define ROBOT_H

	///< Global includes
	#include <MiAMEurobot/MiAMEurobot.h>
	#include <MiAMEurobot/raspberry_pi/RaspberryPi.h>
	#include <math.h>
	#include <stdlib.h>
	#include <stdio.h>
	#include <glib.h>

	#include "uCListener.h"

	typedef struct{
		uCData microcontrollerData;
		L6470 motor;
	}Robot;
	extern Robot robot;

 #endif
