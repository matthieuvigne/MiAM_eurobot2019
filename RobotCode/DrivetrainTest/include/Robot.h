/// \file Robot.h
/// \brief This file defines global variables representing the hardware on the robot, that needs sharing between
///        different files.
///
/// \details This header is included in all other source files. It defines a structure representing the physical robot
///          in the code. Note that these variables are directly available, unprotected: it is to the user to ensure
///          no race condition may occur, for instance when writing the current robot position.
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

	// Right and left macros, for array addressing.
	#define RIGHT 0
	#define LEFT 1

	// Structure containing the current position of the robot.
	typedef struct {
		double x;	///< X coordinate of the robot, in mm.
		double y;	///< Y coordinate of the robot, in mm.
		double theta;	///< Angle of the robot, in rad.
	} RobotPosition;

	typedef struct{
		uCData microcontrollerData; ///< Data structure containing informations from the arduino board.
		L6470 motors[2]; ///< Robot driving motors.
		RobotPosition currentPosition; ///< Current position.
		gboolean isOnRightSide; ///< If we are playing on the right side of the field.
	}Robot;

	extern Robot robot;

	// List of logged values.
		/// \brief List of values to log.
	/// \details Elements of this list determine both the enum values and the header string.
	/// \note Values should be uppercase and start with LOGGER_
	#define LOGGER_VALUES(f) \
		f(LOGGER_TIME)   \
		f(LOGGER_COMMAND_VELOCITY_RIGHT)  \
		f(LOGGER_COMMAND_VELOCITY_LEFT)   \
		f(LOGGER_MOTOR_POSITION_RIGHT)  \
		f(LOGGER_MOTOR_POSITION_LEFT)  \
		f(LOGGER_ENCODER_RIGHT)  \
		f(LOGGER_ENCODER_LEFT)  \
		f(LOGGER_CURRENT_POSITION_X)  \
		f(LOGGER_CURRENT_POSITION_Y)  \
		f(LOGGER_CURRENT_POSITION_THETA)

	#define GENERATE_ENUM(ENUM) ENUM,

	///< Logger field enum.
	typedef enum{
		LOGGER_VALUES(GENERATE_ENUM)
	}LoggerFields;


	// Create header string.
	#define GENERATE_STRING(STRING) #STRING,
	static const char *LOGGER_HEADERS[] = {
		LOGGER_VALUES(GENERATE_STRING)
		NULL
	};

	/// \brief Create a comma-separated formated string list from LoggerFields enum.
	/// \details This string is meant to be given to the Logger object. It consits of the LoggerFields names, lowercase
	///          and without the LOGGER_ prefix.
	/// \return A string of all headers, comma-separated. The returned string should be freed when no longer needed.
	static inline gchar *getHeaderStringList()
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
 #endif
