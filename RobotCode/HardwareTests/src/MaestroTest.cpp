// Test the Maestro servo driver.
#include <MiAMEurobot/MiAMEurobot.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
	std::cout << "Maestro servo driver test." << std::endl;
	std::cout << "Requierments: Maestro servo driver into any usb port of Raspberry Pi ; servo on port 0." << std::endl;

	MaestroDriver maestro;
	if(!maestro.init("/dev/ttyACM0"))
	{
		std::cout << "Could not connect to Maestro servo driver" << std::endl;
		return -1;
	}

	std::cout << "Moving servo 0" << std::endl;
	maestro.setPosition(0, 1000);
	usleep(1000000);
	maestro.setPosition(0, 2000);
	usleep(1000000);
	maestro.setPosition(0, 1500);
	usleep(1000000);
	maestro.setPosition(0, 0);
}
