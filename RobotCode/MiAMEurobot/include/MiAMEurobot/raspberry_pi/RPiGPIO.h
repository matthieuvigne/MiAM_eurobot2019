/// \file raspberry_pi/RPiGPIO.h
/// \brief Access to the Raspberry Pi 3 GPIOs.
///
/// \details GPIO access is done through direct register access, for efficiency. Because the processor changed, this code
///          will not work as-is for older versions of the raspberry. See BCM2837 Peripherals Manual, p.89 onward,
///          for more informations.
///          A quick speed test shows that this code can read and write to this GPIO in under 175ns (2.9MHz frequency).
///          In order to prevent messing up with serial port configuration, only GPIOs 4 to 26 are available.
///          Note that GPIOs 7-11 are used for SPI and 14-15 for UART: setting them as GPIOs would disable the
///          corresponding port.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef RPI_GPIO
#define RPI_GPIO

    typedef enum
    {
      PI_GPIO_OUTPUT               = 0x00,  ///<  Output.
      PI_GPIO_INPUT_PULLUP         = 0x01,   ///<  Input, pullup.
      PI_GPIO_INPUT_PULLDOWN       = 0x02,   ///<  Input, pulldown.
      PI_GPIO_INPUT_NOPULL         = 0x03,   ///<  Input, no pull up/down resistor.
    }
    PiGPIOMode;

    /// Defines for pin direction.
    #define LOW false
    #define HIGH true

    /// \brief Access the GPIO registers.
    ///
    /// \details This function must be called once before any other GPIO function is called.
    ///          Failing to do so would cause the code to crash with a segmentation fault (trying to access undefined memory).
    ///
    /// \return TRUE if memory access was successful, false otherwise.
    bool RPi_enableGPIO();

    /// \brief Setup a GPIO pin as input or output.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \param[in] direction The mode of the GPIO (input, output, with pull resistor settings.
    void RPi_setupGPIO(unsigned int const& gpioPin, PiGPIOMode const& direction);

    /// \brief Write to a GPIO output.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    ///          If the pin is not an output, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \param[in] value FALSE (0) for low, TRUE for high.
    void RPi_writeGPIO(unsigned int const& gpioPin, bool const& value);

    /// \brief Read current GPIO status.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \return The current pin status.
    bool RPi_readGPIO(unsigned int const& gpioPin);
#endif
