# Robot Code
This folder contains all the code of the on-board main processor of the robot: Beaglebone Black (2018 robot) or Raspberry Pi.
This code, written entirely in C, is organished around a library, named MiAMEurobot, that implements all low-level
functionalities needed to use the robot (mainly port activation, motor and sensor drivers...). This library is
generic, and could be reused for other projects.

This folder also contains:
 - TestSensors: a simple sensor test. This code tests all the sensors of the robot, both for code debugging and wiring testing purposes.
 - EurobotCode: the main code of the robot, used for Eurobot itself.
 - DrivetrainTest: code for a simplified drivetrain used to test new components: motors, encoders...
 - HardwareTests: a set of independant tests for the robot hardware (mainly used for wiring tests).

## Compiling this code

This code is packaged in such a way as to be cross-compiled from a PC. Cross-compiling indeed has several advantages, the main
one being that there is no need to copy the source to the Beaglebone, thus preventing any temptation to developed on the board itself
(which can be a bad idea if for instance the Beaglebone dies - due to a USB cable short-circuit, I once lost code that way), and making
it very easy to replace the Beaglebone by any other. Other advantages include faster compilation, and the possibility to test compilation
without any access to the Beaglebone.

This code relies on the [glib](https://developer.gnome.org/glib/) - this choice was made to have a single, unified external library for the whole code.
Thus, to cross-compile this code, one needs:

 - a C cross compiler targeting arm platforms.
 - the glib-2.0 library, cross-compiled.

To obtain this, you may do you own setup, or follow the provided guide in ARMCrossCompilationSetup.pdf.

*Note: this identical setup can be used for cross-compiling C code to a Raspberry Pi, as it is also an arm processor.*


## Compiling and installing the BBBEurobot library.

This library, like the rest of this code, is packaged using cmake. To install it to the folder `<installPath>`, simply run:

```
	cmake /path/to/BBBEurobot -DCMAKE_INSTALL_PREFIX=<installPath>
	make
	make install
```

Once installed, this library can be imported using pkg-config. During install, the pkg-config file `BBBEurobot.pc` is installed in
`<installPath>/lib/pkgconfig` : this path must thus be added to the `PKG_CONFIG_PATH` variable. To do so, open your `.bashrc` file

```
gedit ~/.bashrc
```

and append the following line :

```
export PKG_CONFIG_PATH=<installPath>/lib/pkgconfig
```

Remember to `source ~/.bashrc` before use.

There is a doxygen documentation for this library in `BBBEurobot/doc`. Simply run `make_documentation.sh` to generate it.

## Running code of the Beaglebone.

Obviously, the code we have written aims at using the BeagleBone GPIOs and serial ports. To access these ports, the Beaglebone needs a
device tree overlay (DTO) file to enable these ports.
A DTO file for BBB-Eurobot, enabeling all the ports exposed on the PCB (plus UART4, which is needed to talk to the servo driver - it wasn't planned
on the PCB but a wire can easily be solder to make this connection. Note that you need a 3.3V to 5V converter).

This file is called Eurobot-00A0.dts, and can be found in the library folder (BBB-Eurobot). To use it, copy this file to the Beaglebone,
then compile it and copy it to /lib/firmware:

```
	dtc -O dtb -o Eurobot-00A0.dtbo -b 00 -@ Eurobot-00A0.dts
	cp Eurobot-00A0.dtbo /lib/firmware
```

Now doing ```echo Eurobot > /sys/devices/bone_capemgr.9/slots``` enables all these ports for the duration of the current boot (this is done internally by BBB-Eurobot).
