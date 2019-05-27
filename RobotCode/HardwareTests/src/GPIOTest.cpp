/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
// Test the L6470 stepper motor drivers.
#include <MiAMEurobot/MiAMEurobot.h>
#include <MiAMEurobot/raspberry_pi/RaspberryPi.h>

#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    RPi_enableGPIO();
    std::cout << "Testing GPIO 26 as output." << std::endl;

    RPi_setupGPIO(26, PI_GPIO_OUTPUT);

    while(true)
    {
        RPi_writeGPIO(26, 1);
        usleep(500000);
        RPi_writeGPIO(26, 0);
        usleep(500000);
    }

    return 0;
}

