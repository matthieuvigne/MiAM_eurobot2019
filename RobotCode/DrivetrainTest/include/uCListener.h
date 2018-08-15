/// \file uCListener.h
/// \brief Communication with a slave micro-controller to access some sensors.
///
/// \details This uC constantly broadcast the status of the sensor. This file runs a thread that listen to the serial
///          port, update the value as needed and makes it available through a specific data structure.
///
///	\note	 All functions in this header should be prefixed with uCListener_.
#ifndef ARDUINOSLAVE_H
     #define ARDUINOSLAVE_H

	///< Global includes
	#include <MiAMEurobot/MiAMEurobot.h>

	typedef struct {
		double encoderValues[2]; ///<< Current position of the two encoders, in rad.
	}uCData;

	/// \brief Start the listener thread.
	///
	/// \param[in] portName Name of the serial port to connect to.
	void uCListener_startListener(gchar *portName);

	/// \brief Get the last value read from the sensors.
	uCData uCListener_getData();

 #endif
