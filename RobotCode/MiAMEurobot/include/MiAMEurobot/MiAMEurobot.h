/// \file BBBEurobot.h
/// \brief Master header file.
///
/// \details This header simply includes all the other headers from the library, as well as the glib header,
///          in order to ease library loading.
#ifndef BBB_EUROBOT_DRIVER
#define BBB_EUROBOT_DRIVER

	#include <glib.h>

	#include <MiAMEurobot/KalmanFilter.h>
	#include <MiAMEurobot/Logger.h>
	#include <MiAMEurobot/Metronome.h>

	#include <MiAMEurobot/drivers/ADNS9800Driver.h>
	#include <MiAMEurobot/drivers/I2C-Wrapper.h>
	#include <MiAMEurobot/drivers/IMUDriver.h>
	#include <MiAMEurobot/drivers/L6470Driver.h>
	#include <MiAMEurobot/drivers/LCDDriver.h>
	#include <MiAMEurobot/drivers/MaestroServoDriver.h>
	#include <MiAMEurobot/drivers/PCA9635Driver.h>
	#include <MiAMEurobot/drivers/SPI-Wrapper.h>
	#include <MiAMEurobot/drivers/TCS3472ColorSensorDriver.h>
	#include <MiAMEurobot/drivers/UART-Wrapper.h>

#endif