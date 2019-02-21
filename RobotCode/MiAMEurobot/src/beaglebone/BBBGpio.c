#include "MiAMEurobot/beaglebone/BBBGpio.h"

#include <fstream>
#include <string>

// Internal function : get the content of the first line of the file. Returns TRUE on success, FALSE on error.
bool getFileContent(std::string const& filename, std::string& output)
{
    std::ifstream file(filename);
    if(!file.is_open())
        return false;

    getline(file, output);
    return true;
}

int gpio_digitalRead(int const& pin)
{
    // Get path to gpio direction file.
    std::string filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    // Check file is an input.
    if (filecontent != "in\n")
        return -2;

    // Read gpio value.
    filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    return std::stoi(filecontent);
}

int gpio_digitalWrite(int const& pin, int const& value)
{
    std::string filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    // Check file is an output.
    if (filecontent != "out\n")
        return -2;

    filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream file;
    file.open(filename);
    if(!file.is_open())
        return -1;
    file << (value == 0 ? 0 : 1) << std::endl;
    file.close();
    return 0;
}


int gpio_exportPin(int pin, std::string const& direction)
{
    // Export pin value.
    std::ofstream file;
    file.open("/sys/class/gpio/export", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << pin << std::endl;
    file.close();

    // Set gpio direction.
    file.open("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
    if(!file.is_open())
        return -1;
    file << direction << std::endl;
    file.close();
    return 0;
}


int gpio_analogRead(int const& pin)
{
    if(pin < 0 || pin > 6)
        return -2;

    std::string filename = "/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(pin) + "_raw";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    return std::stoi(filecontent);
}
