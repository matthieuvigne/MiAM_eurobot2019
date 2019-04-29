// Test the USB LCD screen.
#include <MiAMEurobot/raspberry_pi/RaspberryPi.h>
#include <MiAMEurobot/MiAMEurobot.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    std::cout << "VL53L0X rande sensor test." << std::endl;
    std::cout << "Requierments: VL53L0X wired to Raspberry Pi i2c port." << std::endl;

    //~ RPi_enablePorts();
    i2c_open(&RPI_I2C, "/dev/i2c-1");

    VL53L0X sensor;
    if(!sensor.init(&RPI_I2C))
    {
        std::cout << "Could not connect to VL53L0X sensor" << std::endl;
        return -1;
    }
    // Create log
    Logger logger("/tmp/VL53L0XTest.csv", "VL53L0X sensor test", "", "time,distance");
    Metronome metronome(0.010 * 1e9);
    while (true)
    {
        metronome.wait();
        double time = metronome.getElapsedTime();
        double distance = sensor.getMeasurement();
        std::cout << distance << std::endl;
        logger.setData(0, time);
        logger.setData(1, distance);
        logger.writeLine();
    }
}
