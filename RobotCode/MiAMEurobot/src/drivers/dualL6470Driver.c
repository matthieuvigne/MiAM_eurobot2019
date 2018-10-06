#include "MiAMEurobot/drivers/dualL6470Driver.h"
#include "MiAMEurobot/drivers/SPI-Wrapper.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>


// Internal functions: all functions accessible outside of this file are at the end.

// Mutex : for thread safety
static GMutex mut;


// Sends the data contained in the buffer to the bus and reads the incomming
// data from the bus.  The buffer is overwritten with the incoming data.
static int spiReadWrite(dualL6470 l, uint8_t* data, uint8_t len)
{
	if(l.port>0)
		return -1;
	// len represent total message size: split it in packets of 2
	uint8_t nPackets = len / 2;
	g_mutex_lock(&mut);
	l.port = spi_open(l.portName, l.frequency);

	struct spi_ioc_transfer spiCtrl[nPackets];
	// Transmit data in blocks of 2, since two controllers are daisy-chained.
	// This can easily be changed to handle more controllers, but this requires API rethinking.
	for(int x = 0; x < nPackets; x++)
	{
		spiCtrl[x].tx_buf        = (unsigned long)&data[2 * x];
		spiCtrl[x].rx_buf        = (unsigned long)&data[2 * x];
		spiCtrl[x].len           = 2;
		spiCtrl[x].delay_usecs   = 1;
		spiCtrl[x].speed_hz      = l.frequency;
		spiCtrl[x].bits_per_word = 8;
		spiCtrl[x].cs_change = TRUE;
		spiCtrl[x].pad = 0;
		spiCtrl[x].tx_nbits = 0;
		spiCtrl[x].rx_nbits = 0;
	}
	int res = ioctl(l.port, SPI_IOC_MESSAGE(nPackets), &spiCtrl);
	spi_close(l.port);
	g_mutex_unlock(&mut);

    return res;
}


// Send the given command set to the L6470 controllers.
// Length gives the maximum length of both data values (since similar commands are given, they have the same length anyway).
static void sendCommand(dualL6470 l, uint8_t firstCommand, uint32_t firstDataValue, uint8_t secondCommand,
                 uint32_t secondDataValue, uint8_t length, uint32_t *firstResponse, uint32_t *secondResponse)
{
	uint8_t data[2 + 2 * length];
	data[0] = firstCommand;
	data[1] = secondCommand;
	// Fill buffer with given input data
	for(int i = length - 1; i >= 0; i--)
	{
		data[2 * (length - i)] = (firstDataValue >> (8 * i)) & 0xFF;
		data[2 * (length - i) + 1] = (secondDataValue >> (8 * i)) & 0xFF;
	}

	int result = spiReadWrite(l, data, 2 * (length + 1));
	#ifdef DEBUG
		if(result < 0)
			printf("L6470 SPI error: %d %s\n", errno, strerror(errno));
	#endif
	// Decode response, if needed.
	if(firstResponse != NULL && secondResponse != NULL)
	{
		*firstResponse = 0;
		*secondResponse = 0;
		for(uint8_t i = 1; i <= length; i++)
		{
			*firstResponse  += data[2 * i] << (8 * (length - i));
			*secondResponse  += data[2 * i + 1] << (8 * (length - i));
		}
	}
}

// Send a basic command to both motors.
// A basic command is one having no parameter and no return value.
static void sendBasicCommand(dualL6470 l, uint8_t command)
{
	sendCommand(l, command, 0, command, 0, 0, NULL, NULL);
}

// Given a parameter, returns the length of that parameter, in byte
// See Table 9 for parameter length
static uint8_t paramLength(uint8_t param)
{
	switch (param)
	{
		case dSPIN_ABS_POS: return 3;
		case dSPIN_EL_POS: return 2;
		case dSPIN_MARK: return 3;
		case dSPIN_SPEED: return 3;
		case dSPIN_ACC: return 2;
		case dSPIN_DEC: return 2;
		case dSPIN_MAX_SPEED: return 2;
		case dSPIN_MIN_SPEED: return 2;
		case dSPIN_FS_SPD: return 2;
		case dSPIN_INT_SPD: return 2;
		case dSPIN_CONFIG: return 2;
		case dSPIN_STATUS: return 2;
		default: return 1;
	}
}


// Write a specific parameter for both controllers
static void setParam(dualL6470 l, uint8_t param, uint32_t firstValue, uint32_t secondValue)
{
	sendCommand(l, dSPIN_SET_PARAM | param, firstValue,
	               dSPIN_SET_PARAM | param, secondValue, paramLength(param), NULL, NULL);
}

// Get the value of a specific parameter.
void dualL6470_getParam(dualL6470 l, uint8_t param, uint32_t *firstMotorParamOut, uint32_t *secondMotorParamOut)
{
	return sendCommand(l, dSPIN_GET_PARAM | param, 0,
	                      dSPIN_GET_PARAM | param, 0,
	                      paramLength(param), firstMotorParamOut, secondMotorParamOut);
}

void dualL6470_initMotion(dualL6470 l, int maxSpeed, int maxAcceleration)
{
	// Set bridget output to high impedence.
	sendBasicCommand(l, dSPIN_SOFT_HIZ);
    // Reset device.
    sendBasicCommand(l, dSPIN_ACTION_RESET);
    // Set full step mode.
    setParam(l, dSPIN_STEP_MODE, 0, 0);
    sendBasicCommand(l, dSPIN_RESET_POS);
    // Set velocity profile data.
    dualL6470_setVelocityProfile(l, maxSpeed, maxAcceleration, maxAcceleration);
    // Set config param.
    uint32_t configValue = dSPIN_CONFIG_PWM_DIV_1          | dSPIN_CONFIG_PWM_MUL_2
                           | dSPIN_CONFIG_SR_290V_us       | dSPIN_CONFIG_OC_SD_DISABLE
                           | dSPIN_CONFIG_VS_COMP_DISABLE| dSPIN_CONFIG_SW_USER
                           | dSPIN_CONFIG_INT_16MHZ;
    setParam(l, dSPIN_CONFIG, configValue, configValue);
    // Set stall thershold at 2.8A.
    setParam(l, dSPIN_STALL_TH, 90, 90);
    // Set overcurrent detection at 3.4A
    setParam(l, dSPIN_OCD_TH, dSPIN_OCD_TH_3375mA, dSPIN_OCD_TH_3375mA);
}


void dualL6470_initBEMF(dualL6470 l,uint32_t k_hld, uint32_t k_mv, uint32_t int_spd, uint32_t st_slp, uint32_t slp_acc)
{
	// Set all back-EMF parameters.
    setParam(l, dSPIN_KVAL_HOLD, k_hld, k_hld);
    setParam(l, dSPIN_KVAL_ACC, k_mv, k_mv);
    setParam(l, dSPIN_KVAL_DEC, k_mv, k_mv);
    setParam(l, dSPIN_KVAL_RUN, k_mv, k_mv);
    setParam(l, dSPIN_INT_SPD, int_spd, int_spd);
    setParam(l, dSPIN_ST_SLP, st_slp, st_slp);
    setParam(l, dSPIN_FN_SLP_ACC, slp_acc, slp_acc);
    setParam(l, dSPIN_FN_SLP_DEC, slp_acc, slp_acc);
}

void dualL6470_getPosition(dualL6470 l, int32_t *firstPositionOut, int32_t *secondPositionOut)
{
	uint32_t positions[2] = {0, 0};
    dualL6470_getParam(l, dSPIN_ABS_POS, &positions[0], &positions[1]);
    *firstPositionOut = positions[0];
    *secondPositionOut = positions[1];

    // 2s complement
    if(*firstPositionOut > 0x1FFFFF)
        *firstPositionOut = *firstPositionOut + 0xFFC00000;
    if(*secondPositionOut > 0x1FFFFF)
        *secondPositionOut = *secondPositionOut + 0xFFC00000;
}


void dualL6470_getSpeed(dualL6470 l, double *firstVelocityOut, double *secondVelocityOut)
{
    uint32_t firstVelocity, secondVelocity;
    dualL6470_getParam(l, dSPIN_SPEED, &firstVelocity, &secondVelocity);
    double conversionRatio = 0.01490116119;
    *firstVelocityOut = conversionRatio * firstVelocity;
    *secondVelocityOut = conversionRatio * secondVelocity;
}


void dualL6470_setSpeed(dualL6470 l, double firstMotorSpeed, double secondMotorSpeed)
{
	int firstDirection =1;
	if(firstMotorSpeed < 0)
	{
		firstMotorSpeed = -firstMotorSpeed;
		firstDirection = 0;
	}
	int secondDirection =1;
	if(secondMotorSpeed < 0)
	{
		secondMotorSpeed = -secondMotorSpeed;
		secondDirection = 0;
	}

	// Convert to register value, 2s complement.
    uint32_t firstRegister = (firstMotorSpeed * 67.108864) + 0.5;
    uint32_t secondRegister = (secondMotorSpeed * 67.108864) + 0.5;
    if(firstRegister > 0xFFFFF)
		firstRegister = 0xFFFFF;
    if(secondRegister > 0xFFFFF)
		secondRegister = 0xFFFFF;

    sendCommand(l, dSPIN_RUN | firstDirection, firstRegister,
                   dSPIN_RUN | secondDirection, secondRegister, paramLength(dSPIN_SPEED), NULL, NULL);
}


void dualL6470_setVelocityProfile(dualL6470 l, int maxSpeed, int accel, int decel)
{
    sendBasicCommand(l, dSPIN_SOFT_HIZ);

    // Wait for motor to fully stop.
    g_usleep(100000);

	// Set max speed
    float temp = maxSpeed * 0.065536;
    uint32_t regVal = (unsigned long) temp;
    if( regVal > 0x000003FF)
        regVal = 0x000003FF;
    setParam(l, dSPIN_MAX_SPEED, regVal, regVal);

    // Set max acceleration
    temp = accel * 0.068728;
    regVal = (uint32_t) temp;
    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;
    setParam(l, dSPIN_ACC, regVal, regVal);

    // Set max deceleration
    temp = decel * 0.068728;
    regVal = (uint32_t) temp;
    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;
    setParam(l, dSPIN_DEC, regVal, regVal);
}

void dualL6470_softStop(dualL6470 l)
{
    sendBasicCommand(l, dSPIN_SOFT_STOP);
}

void dualL6470_hardStop(dualL6470 l)
{
    sendBasicCommand(l, dSPIN_HARD_STOP);
}


void dualL6470_getStatus(dualL6470 l, uint32_t *firstStatus, uint32_t *secondStatus)
{
	return sendCommand(l, dSPIN_GET_STATUS, 0,
	                      dSPIN_GET_STATUS, 0, paramLength(dSPIN_STATUS), firstStatus, secondStatus);
}


void dualL6470_getError(dualL6470 l, uint32_t *firstError, uint32_t *secondError)
{
    uint32_t status[2];
    dualL6470_getStatus(l, &status[0], &status[1]);
    uint32_t error[2] = {0,0};
    for(int i = 0; i < 2; i++)
    {
		GString *errorMessage = g_string_new("");

		// Not perf cmd is active high, not active low.
		if((status[i] & dSPIN_STATUS_NOTPERF_CMD) != 0)
		{
			error[i] |= dSPIN_ERR_NOEXEC;
			#ifdef DEBUG
				g_string_append(errorMessage, "Cmd no exec ");
			#endif
		}

		// Wrong cmd is active high, not active low.
		if((status[i] & dSPIN_STATUS_WRONG_CMD) != 0)
		{
			error[i] |= dSPIN_ERR_BADCMD;
			#ifdef DEBUG
				g_string_append(errorMessage, "Bad cmd ");
			#endif
		}

		if((status[i] & dSPIN_STATUS_UVLO) == 0)
		{
			error[i] |= dSPIN_ERR_UVLO;
			#ifdef DEBUG
				g_string_append(errorMessage, "Undervoltage ");
			#endif
		}

		if((status[i] & dSPIN_STATUS_TH_SD) == 0)
		{
			error[i] |= dSPIN_ERR_THSHTD;
			#ifdef DEBUG
				g_string_append(errorMessage, "Thermal shutdown ");
			#endif
		}

		if((status[i] & dSPIN_STATUS_OCD) == 0)
		{
			error[i] |= dSPIN_ERR_OVERC;
			#ifdef DEBUG
				g_string_append(errorMessage, "Overcurrent ");
			#endif
		}

		if((status[i] & dSPIN_STATUS_STEP_LOSS_A) == 0)
		{
			//~ error[i] |= dSPIN_ERR_STALLA;
			// Stall is non-verbose as too frequent.
			//~ g_string_append(errorMessage, "Stall A ");
		}

		if((status[i] & dSPIN_STATUS_STEP_LOSS_B) == 0)
		{
			// Stall is non-verbose as too frequent.
			//~ error[i] |= dSPIN_ERR_STALLB;
			//~ g_string_append(errorMessage, "Stall B ");
		}


		#ifdef DEBUG
			if(error[i] > 0)
				printf("dualL6470: %d controller error: %s\n", i, errorMessage->str);
		#endif

		g_string_free(errorMessage, TRUE);
	}
	*firstError = error[0];
	*secondError = error[1];
}


void dualL6470_highZ(dualL6470 l)
{
    sendBasicCommand(l, dSPIN_SOFT_HIZ);
}


void dualL6470_goToPosition(dualL6470 l, int32_t firstPosition, int32_t secondPosition)
{
	int firstDirection = 0;
    if(firstPosition > 0)
		firstDirection = 1;
    uint32_t firstStep = abs(firstPosition);
    if(firstStep > 0x3FFFFF)
		firstStep = 0x3FFFFF;
	int secondDirection = 0;
    if(secondPosition > 0)
		secondDirection = 1;
    uint32_t secondStep = abs(secondPosition);
    if(secondStep > 0x3FFFFF)
		secondStep = 0x3FFFFF;

	sendCommand(l, dSPIN_MOVE | firstDirection, firstStep,
	               dSPIN_MOVE | secondDirection, secondStep, paramLength(dSPIN_ABS_POS), NULL, NULL);
}


gboolean dualL6470_isBusy(dualL6470 l)
{
	uint32_t status[2] = {0, 0};
	dualL6470_getStatus(l, &status[0], &status[1]);

	gboolean busy = FALSE;
	for(int i = 0; i < 2; i++)
	{
		 if((status[i] & dSPIN_STATUS_BUSY) == 0)
		{
			status[i] &= dSPIN_STATUS_MOT_STATUS;
			status[i] >>= 5;
			busy = busy || status[i];
		}
	}

    return busy;
}

void dualL6470_initStructure(dualL6470 *l, const gchar *portName)
{
	// Structure init is the same.
	L6470_initStructure(l, portName);
}
