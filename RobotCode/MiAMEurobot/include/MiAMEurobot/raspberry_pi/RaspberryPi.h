/// \file RaspberryPi.h
/// \brief Master header file for Raspberry Pi.
///
/// \details This header declares Raspberry Pi serial ports and GPIOs, and include functions to access them.
#ifndef RPI_EUROBOT_DRIVER
#define RPI_EUROBOT_DRIVER
	#include <glib.h>
	#include "MiAMEurobot/drivers/I2C-Wrapper.h"
	#include "MiAMEurobot/raspberry_pi/RPiGPIO.h"

	extern const gchar* RPI_SPI_00; ///<  First SPI port.
	extern const gchar* RPI_SPI_01; ///<  Second SPI port.
	extern I2CAdapter RPI_I2C; ///<  I2C port.

	/// \brief Enable the Raspberry Pi serial ports.
	/// \details This function must be called at the start of the program, before any access to a
	///          serial port or gpio. This function returns nothing, but stops the program execution is uncessussful.
	void RPi_enablePorts();
#endif