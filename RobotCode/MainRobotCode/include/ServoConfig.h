/// \file ServoConfig.h
/// \brief Configuration for the main robot servos.

#ifndef SERVO_CONFIG_H
	#define SERVO_CONFIG_H
		/// \brief Servo port definition.
		int const SERVO_TUBE[3] = {0, 1, 2};	// Numbered from right to left.
		int const SERVO_SUCTION[3] = {6, 7, 8};

		int const SERVO_TAP = 12;
		int const SERVO_VERTICAL_TRANSLATION = 13;
		int const PUMP = 14;

		/// \brief Servo position definition.
		int const SP_SUCTION_HIGH = {1500, 1500, 1500};
		int const SP_SUCTION_LOW = {1500, 1500, 1500};

		int const SP_TUBE_OPEN = {1500, 1500, 1500};
		int const SP_TUBE_CLOSE = {1500, 1500, 1500};

		int const SP_TAP_OPEN  = 1000;
		int const SP_TAP_CLOSE  = 1000;

		int const SP_PUMP_OFF = 1500;
		int const SP_PUMP_ON = 2000;
 #endif
