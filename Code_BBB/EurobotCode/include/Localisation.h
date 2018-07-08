/// \file Localisation.h
/// \brief  This file implements a sensor update thread linked to localisation.
///
///
/// \details The function localisation_start() is meant to be run as a separate thread. This
///			 thread runs a fixed-frequency loop, checking and processing sensor values.
///          Its main purpose it to give the position of the robot on the table (hence the name).
///          It is also responsible for checking IR sensors, and of writing a log file.

#ifndef LOCALISATION_H
	#define LOCALISATION_H

	#include <glib.h>

	/// \brief The localisation thread
	///	\details This start the sensor update thread.
	void *localisation_startSensorUpdate();

	/// \brief Reset the position of the robot on the table.
	///
	/// \details This function might be used for example when the robot is put in contact with a side of the table,
	///			 to gain back absolute position accuracy.
	///
	/// \param[in] resetPosition The position to which reset the robot.
	/// \param[in] resetX Wheather or not to reset the X coordinate.
	/// \param[in] resetY Wheather or not to reset the Y coordinate.
	/// \param[in] resetTheta Wheather or not to reset the angle.
	void localisation_reset(RobotPosition resetPosition, gboolean resetX, gboolean resetY, gboolean resetTheta);
#endif
