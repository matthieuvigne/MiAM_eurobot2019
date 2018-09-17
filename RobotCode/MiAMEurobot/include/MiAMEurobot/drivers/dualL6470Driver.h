/// \file dualL6470Driver.h
/// \brief Driver for two L6470 stepper motor drivers daisy-chained on a single SPI port.
///
/// \details Several L6470 drivers can be daisy chained together - this file implements communication
///          with exactly two L6470 (as is the case for instance for the X-NUCLEO-IHM02A1).
///          The API is targetted toward driving a robot chassis, thus constraining some symmetry
///          between both drivers. All functions are thread-safe.
///	\note	 All functions in this header should be prefixed with dualL6470_.
#ifndef dualL6470_DRIVER
#define dualL6470_DRIVER

	#include "MiAMEurobot/drivers/L6470Driver.h"


	///< dualL6470 structure: the same as L6470.
	typedef L6470 dualL6470;


	/// \brief Initialize the dualL6470 structure.
	/// \details See dualL6470_initStructure.
	///
	/// \param[out] l A dualL6470 structure to use to talk to the targeted motor.
    /// \param[in] portName Name of the port, in the file system (i.e. a string "/dev/spidevx").
    /// \param[in] speed Communication speed.
	void dualL6470_initStructure(dualL6470 *l, const gchar *portName);

	/// \brief Initialize both motor drivers to the same parameters.
    ///
    /// \details This function prepares the motor driver for motion, by defining the motor speed, acceleration and
    ///          deceleration. It also sets the step mode and the max stall current. Finally, it places the motor
	///			 in high impedance (i.e. free spinning shaft).
    ///
    /// \note   This function does not initialize the dualL6470 structure.
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] maxSpeed Maximum motor speed, in steps/s (from 15.25 to 15610, resolution 15.25 step/s).
    /// \param[in] maxAcceleration Maximum motor acceleration and deceleration, in steps/s^2 (from 14.55 to 59590,
    ///                            resolution 14.55 step/s^2).
	void dualL6470_initMotion(dualL6470 l, int maxSpeed, int maxAcceleration);


	/// \brief Define BEMF compensation and target current constants for both motors.
    ///
    /// \details This stepper motor includes a back-electromagnetic force compensation. To tune it, it needs informations
    ///		     on the motor. See chip datasheet for more information. ST has released a windows utility to calculate
    ///			 the value of these registers based on information about the motor's parameters and the desired curent.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] k_hld Value of the SPIN_KVAL_HOLD register.
    /// \param[in] k_mv Value of the SPIN_KVAL_ACC, SPIN_KVAL_DEC and SPIN_KVAL_RUN registers.
    /// \param[in] int_spd Value of the SPIN_INT_SPD register.
    /// \param[in] st_slp Value of the SPIN_ST_SLP register.
    /// \param[in] slp_acc Value of the SPIN_FN_SLP_ACC and SPIN_FN_SLP_DEC registers.
	void dualL6470_initBEMF(dualL6470 l, uint32_t k_hld, uint32_t k_mv, uint32_t int_spd, uint32_t st_slp, uint32_t slp_acc);


	/// \brief Read a parameter from both motor controllers.
	///
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] param The parameter to read.
    /// \param[out] firstMotorParamOut Parameter value output for the first motor.
    /// \param[out] secondMotorParamOut Parameter value output for the second motor.
	void dualL6470_getParam(dualL6470 l, uint8_t param, uint32_t *firstMotorParamOut, uint32_t *secondMotorParamOut);


	/// \brief Get current position of both motors.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[out] firstPositionOut Position of the first motor, in steps.
    /// \param[out] secondPositionOut Position of the second motor, in steps.
	void dualL6470_getPosition(dualL6470 l, int32_t *firstPositionOut, int32_t *secondPositionOut);


	/// \brief Get current motor velocity (unsigned, i.e. always positive).
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[out] firstVelocityOut Velocity of the first motor, in steps/s.
    /// \param[out] secondVelocityOut Velocity of the second motor, in steps/s.
	void dualL6470_getSpeed(dualL6470 l, double *firstVelocityOut, double *secondVelocityOut);


	/// \brief Set current motor speed.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] firstMotorSpeed First motor speed, in steps/s (between -15625 and 156250).
    ///            Value will be clamped internally by the maximum speed.
    /// \param[in] secondMotorSpeed Second motor speed, in steps/s.
	void dualL6470_setSpeed(dualL6470 l, double firstMotorSpeed, double secondMotorSpeed);


	/// \brief Set motor max speed and acceleration. The same value is used for both motors.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] maxSpeed Maximum motor speed, in steps/s (from 15.25 to 15610, resolution 15.25 step/s).
    /// \param[in] acc Target motor acceleration, in steps/s^2 (from 14.55 to 59590, resolution 14.55 step/s^2).
    /// \param[in] dec Target motor deceleration, in steps/s^2 (from 14.55 to 59590, resolution 14.55 step/s^2).
	void dualL6470_setVelocityProfile(dualL6470 l, int maxSpeed, int accel, int decel);


	/// \brief Perform a soft motor stop on both motors.
	/// \details A soft stop means that the motor will decelerate at given deceleration until it stops (contrary to a
	///			 hard stop where the motor stops instantaneously). Only soft stop is exposed to prevent damage to the
	///			 motor.
    ///
    /// \param[in] l A valid dualL6470 structure.
	void dualL6470_softStop(dualL6470 l);


	/// \brief Perform a hard motor stop on both motors: motor will try to stop instantaneously.
    /// \note Hard stopping may damage motor or electronics. softStop is the function that should normally be called.
    ///
    /// \param[in] l A valid dualL6470 structure.
	void dualL6470_hardStop(dualL6470 l);


	/// \brief Get the status of both motor controllers.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[out] firstError Status value of the first motor.
    /// \param[out] secondError Status value of the second motor.
	void dualL6470_getStatus(dualL6470 l, uint32_t *firstStatus, uint32_t *secondStatus);


	/// \brief Get the last error read from both motor controller.
	/// \details If an error is present, it will be printed in the terminal.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[out] firstError Error value of the first motor.
    /// \param[out] secondError Error value of the second motor.
	void dualL6470_getError(dualL6470 l, uint32_t *firstError, uint32_t *secondError);


	/// \brief Place both motors in a high impedance state. The shaft will now spin freely.
    ///
    /// \param[in] l A valid dualL6470 structure.
	void dualL6470_highZ(dualL6470 l);


	/// \brief Move both motors a desired number of steps.
    ///
    /// \param[in] l A valid dualL6470 structure.
    /// \param[in] firstPosition The number of steps to move the first motor (negative means backward motion).
    /// \param[in] secondPosition The number of steps to move the second motor (negative means backward motion).
	void dualL6470_goToPosition(dualL6470 l, int32_t firstPosition, int32_t secondPosition);


	/// \brief Returns TRUE if one of the motors is currently moving.
	/// \details This functions can be used to wait for both motors to finish their motion before moving on in the code.
	///
    /// \param[in] l A valid dualL6470 structure.
    /// \returns TRUE if motor is moving.
	gboolean dualL6470_isBusy(dualL6470 l);


#endif
