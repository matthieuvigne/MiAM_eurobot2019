/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
// Test the velocity control scheme of the L6470
// to improve it and understand where the drift comes from.


#include <MiAMEurobot/MiAMEurobot.h>
#include <MiAMEurobot/raspberry_pi/RaspberryPi.h>

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <string>
#include <algorithm>

using miam::RobotPosition;

// Update loop frequency
const double LOOP_PERIOD = 0.010;

// Reduction ratio
const double MM_TO_STEP = 600 / (2 * M_PI * 50.0);


// Logger-related variables.
/// \brief List of values to log.
/// \details Elements of this list determine both the enum values and the header string.
/// \note Values should be uppercase and start with LOGGER_
#define LOGGER_VALUES(f) \
    f(LOGGER_TIME)   \
    f(LOGGER_TRAJECTORY_POSITION)  \
    f(LOGGER_TRAJECTORY_VELOCITY)  \
    f(LOGGER_MOTOR_POSITION)   \
    f(LOGGER_MOTOR_POSITION_ERROR)   \
    f(LOGGER_MOTOR_VELOCITY)   \
    f(LOGGER_MOTOR_VELOCITY_ERROR)   \
    f(LOGGER_EULER_POSITION)   \
    f(LOGGER_EULER_POSITION_ERROR)   \

#define GENERATE_ENUM(ENUM) ENUM,

///< Logger field enum.
typedef enum{
    LOGGER_VALUES(GENERATE_ENUM)
}LoggerFields;


// Create header string.
#define GENERATE_STRING(STRING) #STRING,
static const char *loggerHEADERS[] = {
    LOGGER_VALUES(GENERATE_STRING)
    NULL
};

// Process input string to go from enum name to string name.
std::string enumToHeaderString(std::string const& s)
{
    std::string outputString = s;
    // Remove logger prefix.
    outputString = outputString.substr(7);
    // Put it in lowercase.
    std::transform(outputString.begin(), outputString.end(), outputString.begin(), ::tolower);
    //Iterate over string, looking for underscores.
    for(unsigned int i = 0; i < outputString.length(); i++)
    {
        if(outputString[i] == '_')
        {
            outputString.erase(i, 1);
            outputString[i] = std::toupper(outputString[i]);
        }
    }
    return outputString;
}
/// \brief Create a comma-separated formated string list from LoggerFields enum.
/// \details This string is meant to be given to the Logger object. It consits of the LoggerFields names, lowercase
///          and without the logger prefix.
/// \return A string of all headers, comma-separated. The returned string should be freed when no longer needed.
static inline std::string getHeaderStringList()
{
    std::string headerString = enumToHeaderString(loggerHEADERS[0]);
    int i = 1;
    while(loggerHEADERS[i] != NULL)
    {
        headerString = headerString + "," + enumToHeaderString(loggerHEADERS[i]);
        i++;
    }
    return headerString;
}


// Maximum motor speed and acceleration.
int maxSpeed = 600;
int maxAcceleration = 600;

// Motor current input value.
// These are the default value of the L6470.
const int MOTOR_KVAL_HOLD = 0x29;
const int MOTOR_BEMF[4] = {0x29, 0x0408, 0x19, 0x29};

int main(int argc, char **argv)
{
    std::cout << "L6470 control test." << std::endl;
    std::cout << "Requierments: X-NUCLEO-IHM02A1 wired to SPI port 0 of Raspberry Pi." << std::endl;

    // Init connection with driver.
    miam::L6470 stepperMotors(RPI_SPI_00, 2);
    bool areMotorsInit = stepperMotors.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD, MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
    if(areMotorsInit)
        std::cout << "Connection with driver successful." << std::endl;
    else
    {
        std::cout << "Failed to communicate with L6470 board" << std::endl;
        //~ return -1;
    }

    // Generate trapezoid trajectory.
    RobotPosition startPoint;
    RobotPosition endPoint;
    endPoint.x = 500;
    miam::trajectory::StraightLine line(startPoint, endPoint);

    // Start logger.
    Logger logger("velocityTest.csv", "Test velocity control", "control test", getHeaderStringList());

    std::vector<int32_t> motorPosition;
    motorPosition.push_back(0);
    motorPosition.push_back(0);
    std::vector<double> motorSpeed;
    motorSpeed.push_back(0);
    motorSpeed.push_back(0);

    double lastTime = 0.0;
    double lastTarget = 0.0;
    double lastVelocity = 0.0;

    double eulerIntegrationPosition = 0.0;
    // Start metronome.
    Metronome metronome(LOOP_PERIOD * 1e9);


    std::vector<double> velocities;
    //~ velocities.push_back(p.linearVelocity * MM_TO_STEP);
    //~ velocities.push_back(5000);
    velocities.push_back(200);
    velocities.push_back(200);
    //~ logger.setData(LOGGER_TRAJECTORY_POSITION, lastTarget);
    //~ logger.setData(LOGGER_TRAJECTORY_VELOCITY, velocities[0]);
    stepperMotors.setSpeed(velocities);

    std::vector<uint32_t> conf = stepperMotors.getParam(0x18);

    std::cout << conf[0] << std::endl;
    std::cout << conf[1] << std::endl;

    //~ while(metronome.getElapsedTime() < line.getDuration() + 1.0)
    while(metronome.getElapsedTime() < 20.0)
    {
        // Wait for next tick.
        metronome.wait();
        double time = metronome.getElapsedTime();
        // Log current motor position and velocity.
        motorPosition = stepperMotors.getPosition();
        motorSpeed = stepperMotors.getSpeed();
        std::cout << motorPosition[0] << " " << motorPosition[1] << std::endl;
        std::cout << motorSpeed[0] << " " << motorSpeed[1] << std::endl;
        logger.setData(LOGGER_MOTOR_POSITION, motorPosition[1]);
        logger.setData(LOGGER_MOTOR_POSITION_ERROR, motorPosition[1] - lastTarget);
        logger.setData(LOGGER_MOTOR_VELOCITY, motorSpeed[1]);
        logger.setData(LOGGER_MOTOR_VELOCITY_ERROR, motorSpeed[1] - lastVelocity);

        // Create new log line with new target and timestamp.
        logger.writeLine();
        logger.setData(LOGGER_TIME, time);

        miam::trajectory::TrajectoryPoint p = line.getCurrentPoint(time);

        lastTarget = p.position.x * MM_TO_STEP;
        std::vector<double> velocities;
        //~ velocities.push_back(p.linearVelocity * MM_TO_STEP);
        velocities.push_back(100);
        velocities.push_back(0);
        logger.setData(LOGGER_TRAJECTORY_POSITION, lastTarget);
        logger.setData(LOGGER_TRAJECTORY_VELOCITY, velocities[1]);
        //~ stepperMotors.setSpeed(velocities);

        // Perform euler integration of the sent velocity.
        eulerIntegrationPosition += (time - lastTime) * velocities[1];
        lastTime = time;
        lastVelocity = p.linearVelocity * MM_TO_STEP;
        logger.setData(LOGGER_EULER_POSITION, eulerIntegrationPosition);
        logger.setData(LOGGER_EULER_POSITION_ERROR, eulerIntegrationPosition - lastTarget);

    }
    return 0;
}

