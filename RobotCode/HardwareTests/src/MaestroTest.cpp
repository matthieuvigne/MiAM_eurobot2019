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

    int servoToTest = 14;
    std::cout << "Moving servo " << servoToTest << std::endl;
    //~ maestro.setPosition(servoToTest, 1050);
    //~ usleep(1000000);
    //~ maestro.setPosition(servoToTest, 1980);
    //~ usleep(10000000);
    //~ maestro.setPosition(servoToTest, 1500);
    //~ usleep(1000000);
    //~ maestro.setPosition(servoToTest, 0);

    maestro.setPosition(14, 1980);

    maestro.setPosition(6, 1840);
    maestro.setPosition(8, 1850);
    usleep(10000000);
    maestro.setPosition(14, 0);

    maestro.setPosition(6, 1600);
    maestro.setPosition(8, 1600);
}
