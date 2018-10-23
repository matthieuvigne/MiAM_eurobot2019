#include "MiAMEurobot/drivers/L6470Driver.h"
#include "MiAMEurobot/drivers/SPI-Wrapper.h"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>

// Register definition and utility function for parameter length.
#include "L6470Registers.h"

namespace miam
{
	L6470::L6470():
		portName_(""),
		numberOfDevices_(0),
		frequency_(0)
	{

	}


	L6470::L6470(std::string const& portName, int const& numberOfDevices, int const& busFrequency):
		portName_(portName),
		numberOfDevices_(std::abs(numberOfDevices)),
		frequency_(std::abs(busFrequency))
	{

	}


	L6470& L6470::operator=(L6470 const& l)
	{
		portName_ = l.portName_;
		numberOfDevices_ = l.numberOfDevices_;
		frequency_ = l.frequency_;
		return *this;
	}


	bool L6470::init(uint32_t const& maxSpeed, uint32_t const& maxAcceleration, uint32_t const& k_hld,
	                 uint32_t const& k_mv, uint32_t  const& int_spd, uint32_t const& st_slp, uint32_t const& slp_acc)
	{
		// Stop motors, reset devices.
		highZ();
		sendCommand(dSPIN_ACTION_RESET);
		// Set full step mode.
		setParam(dSPIN_STEP_MODE, 0);
		sendCommand(dSPIN_RESET_POS);
		setVelocityProfile(maxSpeed, maxAcceleration, maxAcceleration);

		// Set config param.
		uint32_t configValue = dSPIN_CONFIG_PWM_DIV_1          | dSPIN_CONFIG_PWM_MUL_2
							   | dSPIN_CONFIG_SR_290V_us       | dSPIN_CONFIG_OC_SD_DISABLE
							   | dSPIN_CONFIG_VS_COMP_DISABLE| dSPIN_CONFIG_SW_USER
							   | dSPIN_CONFIG_INT_16MHZ;
		setParam(dSPIN_CONFIG, configValue);
		// Set stall thershold at 2.8A.
		setParam(dSPIN_STALL_TH, 90);
		// Set overcurrent detection at 2.6A.
		setParam(dSPIN_OCD_TH, dSPIN_OCD_TH_2625mA);

		// Set back-emf constants.
		setParam(dSPIN_KVAL_HOLD, k_hld);
		setParam(dSPIN_KVAL_ACC, k_mv);
		setParam(dSPIN_KVAL_DEC, k_mv);
		setParam(dSPIN_KVAL_RUN, k_mv);
		setParam(dSPIN_INT_SPD, int_spd);
		setParam(dSPIN_ST_SLP, st_slp);
		setParam(dSPIN_FN_SLP_ACC, slp_acc);
		setParam(dSPIN_FN_SLP_DEC, slp_acc);

		// Verify that parameter setting worked.
		std::vector<uint32_t> value = getParam(dSPIN_KVAL_HOLD);
		bool isInitSuccessful = true;
		for(uint32_t v : value)
			if(v != k_hld)
			{
				isInitSuccessful = false;
				break;
			}
		return isInitSuccessful;
	}


	void L6470::setParam(uint8_t const& param, std::vector<uint32_t> const& parameterValues)
	{
		std::vector<uint8_t> commands;
		for(uint i = 0; i < numberOfDevices_; i++)
			commands.push_back(dSPIN_SET_PARAM | param);
		sendCommand(commands, parameterValues, getParamLength(param));
	}


	void L6470::setParam(uint8_t const& param, uint32_t const& parameterValue)
	{
		std::vector<uint32_t> values;
		for(uint i = 0; i < numberOfDevices_; i++)
			values.push_back(parameterValue);
		setParam(param, values);
	}


	std::vector<uint32_t> L6470::getParam(uint8_t const& param)
	{
		std::vector<uint8_t> commands;
		std::vector<uint32_t> parameterValue;
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			commands.push_back(dSPIN_GET_PARAM | param);
			parameterValue.push_back(0.0);
		}
		return sendCommand(commands, parameterValue, getParamLength(param));
	}


	void L6470::softStop()
	{
		sendCommand(dSPIN_SOFT_STOP);
	}


	void L6470::hardStop()
	{
		sendCommand(dSPIN_HARD_STOP);
	}


	void L6470::highZ()
	{
		sendCommand(dSPIN_SOFT_HIZ);
	}


	std::vector<int32_t> L6470::getPosition()
	{
		std::vector<int32_t> positions;

		std::vector<uint32_t> value = getParam(dSPIN_ABS_POS);
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			positions.push_back(value[i]);
			// 2s complement
			if(positions[i] > 0x1FFFFF)
				positions[i] = positions[i] + 0xFFC00000;
		}
		return positions;
	}


	std::vector<double> L6470::getSpeed()
	{
		std::vector<double> speeds;
		// Register value to steps/s.
		double const conversionRatio = 0.01490116119;

		std::vector<uint32_t> value = getParam(dSPIN_SPEED);
		for(uint i = 0; i < numberOfDevices_; i++)
			speeds.push_back(conversionRatio * value[i]);
		return speeds;
	}


	void L6470::setSpeed(std::vector<double> const& motorSpeeds)
	{
		std::vector<uint8_t> commands;
		std::vector<uint32_t> parameters;
		// Fill command and parameters registers.
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			double speed = 0.0;
			if(i < motorSpeeds.size())
				speed = motorSpeeds[i];

			// Command: run. Direction is indicated there.
			uint8_t command = dSPIN_RUN;
			if(speed >= 0.0)
				command |= 1;
			commands.push_back(command);

			// Register value.
			uint32_t registerValue = (std::abs(speed) * 67.108864) + 0.5;
			// Clamp
			if(registerValue > 0xFFFFF)
				registerValue = 0xFFFFF;
			parameters.push_back(registerValue);
		}
		sendCommand(commands, parameters, getParamLength(dSPIN_SPEED));
	}


	void L6470::setVelocityProfile(double const& maxSpeed, double const& accel, double const& decel)
	{
		sendCommand(dSPIN_SOFT_HIZ);

		// Wait for motor to fully stop.
		g_usleep(100000);

		// Set max speed
		float temp = maxSpeed * 0.065536;
		uint32_t regVal = (unsigned long) temp;
		if( regVal > 0x000003FF)
			regVal = 0x000003FF;
		setParam(dSPIN_MAX_SPEED, regVal);

		// Set max acceleration
		temp = accel * 0.068728;
		regVal = (uint32_t) temp;
		if(regVal > 0x00000FFF)
			regVal = 0x00000FFF;
		setParam(dSPIN_ACC, regVal);

		// Set max deceleration
		temp = decel * 0.068728;
		regVal = (uint32_t) temp;
		if(regVal > 0x00000FFF)
			regVal = 0x00000FFF;
		setParam(dSPIN_DEC, regVal);
	}


	std::vector<uint32_t> L6470::getError()
	{
		std::vector<uint32_t> status = getStatus();
		std::vector<uint32_t> errors;
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			uint32_t error = 0.0;
			GString *errorMessage = g_string_new("");

			// Not perf cmd is active high, not active low.
			if((status[i] & dSPIN_STATUS_NOTPERF_CMD) != 0)
			{
				error |= dSPIN_ERR_NOEXEC;
				#ifdef DEBUG
					g_string_append(errorMessage, "Cmd no exec ");
				#endif
			}

			// Wrong cmd is active high, not active low.
			if((status[i] & dSPIN_STATUS_WRONG_CMD) != 0)
			{
				error |= dSPIN_ERR_BADCMD;
				#ifdef DEBUG
					g_string_append(errorMessage, "Bad cmd ");
				#endif
			}

			if((status[i] & dSPIN_STATUS_UVLO) == 0)
			{
				error |= dSPIN_ERR_UVLO;
				#ifdef DEBUG
					g_string_append(errorMessage, "Undervoltage ");
				#endif
			}

			if((status[i] & dSPIN_STATUS_TH_SD) == 0)
			{
				error |= dSPIN_ERR_THSHTD;
				#ifdef DEBUG
					g_string_append(errorMessage, "Thermal shutdown ");
				#endif
			}

			if((status[i] & dSPIN_STATUS_OCD) == 0)
			{
				error |= dSPIN_ERR_OVERC;
				#ifdef DEBUG
					g_string_append(errorMessage, "Overcurrent ");
				#endif
			}

			if((status[i] & dSPIN_STATUS_STEP_LOSS_A) == 0)
			{
				//~ // Stall is non-verbose as too frequent.
				error |= dSPIN_ERR_STALLA;
				g_string_append(errorMessage, "Stall A ");
			}

			if((status[i] & dSPIN_STATUS_STEP_LOSS_B) == 0)
			{
				//~ // Stall is non-verbose as too frequent.
				error |= dSPIN_ERR_STALLB;
				g_string_append(errorMessage, "Stall B ");
			}


			#ifdef DEBUG
				if(error > 0)
					printf("dualL6470: %d controller error: %s\n", i, errorMessage->str);
			#endif

			g_string_free(errorMessage, TRUE);
			errors.push_back(error);
		}
		return errors;
	}


	bool L6470::isBusy()
	{
		std::vector<uint32_t> status = getStatus();
		bool busy = false;
		for(uint i = 0; i < numberOfDevices_; i++)
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


	void L6470::moveNSteps(std::vector<int32_t> nSteps)
	{
		std::vector<uint8_t> commands;
		std::vector<uint32_t> parameters;
		// Fill command and parameters registers.
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			int32_t nStep = 0;
			if(i < nSteps.size())
				nStep = nSteps[i];

			// Command: move. Direction is indicated there.
			uint8_t command = dSPIN_MOVE;
			if(nStep >= 0)
				command |= 1;
			commands.push_back(command);

			// Register value.
			uint32_t registerValue = std::abs(nSteps[i]);
			// Clamp
			if(registerValue > 0x3FFFFF)
				registerValue = 0x3FFFFF;
			parameters.push_back(registerValue);
		}
		sendCommand(commands, parameters, getParamLength(dSPIN_ABS_POS));
	}


	int L6470::spiReadWrite(uint8_t* data, uint8_t const& len)
	{
		// len represent total message size: split it in packets of numberOfDevices_
		uint8_t nPackets = len / numberOfDevices_;
		mutex_.lock();
		int port = spi_open(portName_.c_str(), frequency_);

		struct spi_ioc_transfer spiCtrl[nPackets];
		// Transmit data in blocks of 2, since two controllers are daisy-chained.
		// This can easily be changed to handle more controllers, but this requires API rethinking.
		for(int x = 0; x < nPackets; x++)
		{
			spiCtrl[x].tx_buf        = (unsigned long)&data[2 * x];
			spiCtrl[x].rx_buf        = (unsigned long)&data[2 * x];
			spiCtrl[x].len           = numberOfDevices_;
			spiCtrl[x].delay_usecs   = 1;
			spiCtrl[x].speed_hz      = frequency_;
			spiCtrl[x].bits_per_word = 8;
			spiCtrl[x].cs_change = TRUE;
			spiCtrl[x].pad = 0;
			spiCtrl[x].tx_nbits = 0;
			spiCtrl[x].rx_nbits = 0;
		}
		int res = ioctl(port, SPI_IOC_MESSAGE(nPackets), &spiCtrl);
		spi_close(port);
		mutex_.unlock();

		return res;
	}


	std::vector<uint32_t> L6470::sendCommand(std::vector<uint8_t> const& commands, std::vector<uint32_t> const& parameters, uint8_t paramLength)
	{
		std::vector<uint32_t> response;
		for(uint i = 0; i < numberOfDevices_; i++)
			response.push_back(0.0);

		// Check length of input vector
		if(commands.size() != numberOfDevices_)
			return response;
		if(paramLength > 0)
			if(parameters.size() != numberOfDevices_)
				return response;

		// Create data vector to send.
		uint8_t data[numberOfDevices_ * (1 + paramLength)];
		// Fill buffer with given input data
		for(uint i = 0; i < numberOfDevices_; i++)
			data[i] = commands[i];
		for(int i = paramLength - 1; i >= 0; i--)
		{
			for(uint j = 0; j < numberOfDevices_; j++)
				data[numberOfDevices_ * (paramLength - i) + j] = (parameters.at(j) >> (8 * i)) & 0xFF;
		}

		// Do SPI communication
		int result = spiReadWrite(data, numberOfDevices_ * (1 + paramLength));
		#ifdef DEBUG
			if(result < 0)
				printf("L6470 SPI error: %d %s\n", errno, strerror(errno));
		#endif
		// Decode response.

		for(uint8_t i = 1; i <= paramLength; i++)
		{
			for(uint j = 0; j < numberOfDevices_; j++)
				response.at(j)  += data[numberOfDevices_ * i] << (8 * (paramLength - i));
		}
		return response;
	}


	void L6470::sendCommand(uint8_t const& command)
	{
		std::vector<uint8_t> commands;
		for(uint i = 0; i < numberOfDevices_; i++)
			commands.push_back(command);
		std::vector<uint32_t> parameters;
		sendCommand(commands, parameters, 0.0);
	}


	std::vector<uint32_t> L6470::getStatus()
	{

		std::vector<uint8_t> commands;
		std::vector<uint32_t> params;
		for(uint i = 0; i < numberOfDevices_; i++)
		{
			commands.push_back(dSPIN_GET_STATUS);
			params.push_back(0);
		}
		return sendCommand(commands, params, getParamLength(dSPIN_STATUS));
	}
}
