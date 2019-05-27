/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
// Test the RPLidar interface.
#include <MiAMEurobot/drivers/USBLCDDriver.h>
#include "MiAMEurobot/RPLidarHandler.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    std::cout << "RPLIDAR A2 test." << std::endl;
    std::cout << "Requierments: RPLIDAR plugged into any usb port of Raspberry Pi." << std::endl;

    RPLidarHandler lidar;
    if(!lidar.init("/dev/ttyUSB0"))
    {
        std::cout << "Failed to talk to RPLidar" << std::endl;
        return -1;
    }

    for(int i = 0; i < 10; i ++)
    {
        lidar.update();
        std::cout << lidar.debuggingBuffer_[lidar.debuggingBufferPosition_].theta << " " << lidar.debuggingBuffer_[lidar.debuggingBufferPosition_].r << std::endl;
        usleep(1000000);
    }
    return 0;
}
