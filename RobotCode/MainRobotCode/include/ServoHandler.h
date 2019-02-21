/// \file ServoConfig.h
/// \brief Configuration for the main robot servos.

#ifndef SERVO_CONFIG_H
	#define SERVO_CONFIG_H
		#include <MiAMEurobot/drivers/MaestroServoDriver.h>

		/// \brief Helper class for controlling the robot servos.
		class ServoHandler
		{
			public:
				/// \brief Constructor
				ServoHandler();

				/// \brief Initialize communication with the Maestro servo driver.
				///
				/// \param portName Serial port file name ("/dev/ttyOx")
				/// \returns   true on success, false otherwise.
				bool init(std::string const& portName);

				void openTube(int tubeNumber); ///< Open suction air tube.
				void closeTube(int tubeNumber); ///< Close suction air tube.

			private:
				MaestroDriver maestro_;
		};
 #endif
