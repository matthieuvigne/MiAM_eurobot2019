# Electronics

This folder contains the general wiring diagram of the robot, along with schematics and PCB files for custom boards,
realised with Kicad.

 - RPiShield: a somewhat generic shield ("hat") for the Raspberry Pi, to make GPIO and serial connections easier. This
 shield exposes, in a well-ordered manner:
	- the I2C port, with several pins to easily wire multiple salves.
	- three SPI ports (SPI0.0, SPI0.1 and SPI1.2), with a buffer protecting the CS line (useful for the L6470 breakout board,
	where this line is driven high at startup, possibly before the Raspberry Pi is powered up).
	- the UART port.
	- both hardware PWM pins.
	- three onboard LEDs.
	- the remaining 8 GPIOs.
 Note that this shield does not include any buffering on GPIOs.

 - library: a folder containing a Kicad library "MiAM", containing random custom parts used in these boards.
