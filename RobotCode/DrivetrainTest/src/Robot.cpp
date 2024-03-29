#include "Robot.h"

#include <iostream>

#include <algorithm>
#include <string>

// Update loop frequency
const double LOOP_PERIOD = 0.010;

// Logger-related variables.
/// \brief List of values to log.
/// \details Elements of this list determine both the enum values and the header string.
/// \note Values should be uppercase and start with LOGGER_
#define LOGGER_VALUES(f) \
    f(LOGGER_TIME)   \
    f(LOGGER_COMMAND_VELOCITY_RIGHT)  \
    f(LOGGER_COMMAND_VELOCITY_LEFT)   \
    f(LOGGER_MOTOR_POSITION_RIGHT)  \
    f(LOGGER_MOTOR_POSITION_LEFT)  \
    f(LOGGER_ENCODER_RIGHT)  \
    f(LOGGER_ENCODER_LEFT)  \
    f(LOGGER_CURRENT_POSITION_X)  \
    f(LOGGER_CURRENT_POSITION_Y)  \
    f(LOGGER_CURRENT_POSITION_THETA) \
    f(LOGGER_TARGET_POSITION_X)  \
    f(LOGGER_TARGET_POSITION_Y)  \
    f(LOGGER_TARGET_POSITION_THETA) \
    f(LOGGER_TARGET_LINEAR_VELOCITY) \
    f(LOGGER_TARGET_ANGULAR_VELOCITY) \
    f(LOGGER_TRACKING_LONGITUDINAL_ERROR) \
    f(LOGGER_TRACKING_TRANSVERSE_ERROR) \
    f(LOGGER_TRACKING_ANGLE_ERROR) \

#define GENERATE_ENUM(ENUM) ENUM,

///< Logger field enum.
typedef enum{
    LOGGER_VALUES(GENERATE_ENUM)
}LoggerFields;


// Create header string.
#define GENERATE_STRING(STRING) #STRING,
static const char *LOGGER_HEADERS[] = {
    LOGGER_VALUES(GENERATE_STRING)
    NULL
};

// Process input string to go from enum name to string name.
std::string enumToHeaderString(std::string const& s)
{
    std::string outputString = s;
    // Remove logger_ prefix.
    outputString = outputString.substr(7);
    // Put it in lowercase.
    std::transform(outputString.begin(), outputString.end(), outputString.begin(), ::tolower);
    //Iterate over string, looking for underscores.
    for(int i = 0; i < outputString.length(); i++)
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
///          and without the LOGGER_ prefix.
/// \return A string of all headers, comma-separated. The returned string should be freed when no longer needed.
static inline std::string getHeaderStringList()
{
    std::string headerString = enumToHeaderString(LOGGER_HEADERS[0]);
    int i = 1;
    while(LOGGER_HEADERS[i] != NULL)
    {
        headerString = headerString + "," + enumToHeaderString(LOGGER_HEADERS[i]);
        i++;
    }
    return headerString;
}


Robot::Robot():
    currentTime_(0.0)
{
    motorSpeed_.push_back(0.0);
    motorSpeed_.push_back(0.0);
    motorPosition_.push_back(0);
    motorPosition_.push_back(0);
}

bool Robot::init()
{
    // Create logger.
    gchar *date = g_date_time_format(g_date_time_new_now_local(), "%Y%m%dT%H%M%SZ");
    gchar *filename = g_strdup_printf("log%s.csv", date);
    g_free(date);
    std::string headers = getHeaderStringList();
    // Log robot dimensions in header.
    std::string info = "wheelRadius:" + std::to_string(robotdimensions::wheelRadius) + \
                        "_wheelSpacing:" + std::to_string(robotdimensions::wheelSpacing) + \
                        "_stepSize:" + std::to_string(robotdimensions::stepSize);

    logger_ = Logger(filename, "Drivetrain test robot", info, getHeaderStringList());
    g_free(filename);

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 0;
    initialPosition.y = 0;
    initialPosition.theta = 0;
    currentPosition_.set(initialPosition);

    // Set PIDs.
    PIDLinear_ = miam::PID(controller::linearKp, controller::linearKd, controller::linearKi, 0.2);
    PIDAngular_ = miam::PID(controller::rotationKp, controller::rotationKd, controller::rotationKi, 0.15);

    return true;
}

RobotPosition Robot::getCurrentPosition()
{
    return currentPosition_.get();
}

void Robot::resetPosition(RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta)
{
    RobotPosition position = currentPosition_.get();
    if(resetX)
        position.x = resetPosition.x;
    if(resetY)
        position.y = resetPosition.y;
    if(resetTheta)
        position.theta = resetPosition.theta;
    currentPosition_.set(position);
}


void Robot::setTrajectoryToFollow(std::vector<std::shared_ptr<Trajectory>> const& trajectories)
{
    newTrajectories_ = trajectories;
}

bool Robot::waitForTrajectoryFinished()
{
    while(currentTrajectories_.size() > 0)
        g_usleep(2.0 * 1000000 * LOOP_PERIOD);
    return TRUE;
}


bool Robot::followTrajectory(Trajectory *traj, double const& timeInTrajectory, double const & dt)
{
    // Get current trajectory state.
    trajectoryPoint_ = traj->getCurrentPoint(timeInTrajectory);

    // Compute targets for rotation and translation motors.
    double translationMotor, rotationMotor;

    // Feedforward.
    translationMotor = trajectoryPoint_.linearVelocity;
    rotationMotor = trajectoryPoint_.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_.get();
    RobotPosition error = currentPosition - trajectoryPoint_.position;

    // Minus of y error to work back in a "standard" frame.
    error.y = -error.y;

    // Project along line of slope theta.
    RobotPosition tangent(std::cos(trajectoryPoint_.position.theta), std::sin(trajectoryPoint_.position.theta), 0.0);
    RobotPosition orthogonal(-std::sin(trajectoryPoint_.position.theta), std::cos(trajectoryPoint_.position.theta), 0.0);
    RobotPosition colinear, normal;
    error.projectOnto(tangent, colinear, normal);

    trackingLongitudinalError_ = colinear.dot(tangent);

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 && std::abs(trajectoryPoint_.angularVelocity) > 1e-4)
        trackingLongitudinalError_ = 0;

    trackingTransverseError_ = normal.dot(orthogonal);
    // Change sign if going backward.
    if(trajectoryPoint_.linearVelocity < 0)
        trackingTransverseError_ = - trackingTransverseError_;
    trackingAngleError_ = currentPosition.theta - trajectoryPoint_.position.theta;

    // If we are beyon trajector end, look to see if we are close enough to the target point to stop.
    if(traj->getDuration() <= timeInTrajectory)
    {
        if(trackingLongitudinalError_ < 3 && trackingAngleError_ < 0.02 && motorSpeed_[RIGHT] < 100 && motorSpeed_[LEFT] < 100)
        {
            // Just stop the robot.
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            return true;
        }
    }

    // Compute correction terms.
    translationMotor += PIDLinear_.computeValue(trackingLongitudinalError_, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError_;
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 * robotdimensions::maxWheelSpeed)
        angularPIDError += controller::transverseKp * trajectoryPoint_.linearVelocity / robotdimensions::maxWheelSpeed * trackingTransverseError_;
    rotationMotor += PIDAngular_.computeValue(angularPIDError, dt);

    // Compute the resulting velocity of each wheel, in mm/s.
    double wheelVelocity[2];
    wheelVelocity[RIGHT] = translationMotor + robotdimensions::wheelSpacing * rotationMotor;
    wheelVelocity[LEFT] = translationMotor - robotdimensions::wheelSpacing * rotationMotor;

    for(int i = 0; i < 2; i++)
        motorSpeed_[i] = wheelVelocity[i] / robotdimensions::wheelRadius / robotdimensions::stepSize;

    return false;
}

void Robot::updateTrajectoryFollowingTarget(double const& dt)
{
    // Load new trajectories, if needed.
    if(!newTrajectories_.empty())
    {
        // We have new trajectories, erase the current trajectories and follow the new one.
        currentTrajectories_ = newTrajectories_;
        trajectoryStartTime_ = currentTime_;
        newTrajectories_.clear();
        std::cout << "Recieved new trajectory" << std::endl;
    }

    // If we have no trajectory to follow, do nothing.
    if(currentTrajectories_.empty())
    {
        motorSpeed_[0] = 0.0;
        motorSpeed_[1] = 0.0;
    }
    else
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        // Look if first trajectory is done.
        double timeInTrajectory = currentTime_ - trajectoryStartTime_;
        if(timeInTrajectory > traj->getDuration())
        {
            // If we still have a trajectory after that, immediately switch to the next trajectory.
            if(currentTrajectories_.size() > 1)
            {
                // Not obtimal, could be improved.
                currentTrajectories_.erase(currentTrajectories_.begin());
                traj = currentTrajectories_.at(0).get();
                trajectoryStartTime_ = currentTime_;
                timeInTrajectory = 0.0;
                std::cout << "Trajectory done, going to next one" << std::endl;
            }
        }
        // If we are more than 1 second after the end of the trajectory, stop it anyway.
        // We hope to have servoed the robot is less than that anyway.
        if(timeInTrajectory - 1.0 >  traj->getDuration())
        {
            std::cout << "Timeout on trajectory following" << std::endl;
            currentTrajectories_.erase(currentTrajectories_.begin());
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
        }
        else
        {
            // Servo robot on current trajectory.
            bool trajectoryDone = followTrajectory(traj, timeInTrajectory, dt);
            // If we finished the last trajectory, we can just end it straight away.
            if(trajectoryDone && currentTrajectories_.size() == 1)
            {
                currentTrajectories_.erase(currentTrajectories_.begin());
                motorSpeed_[0] = 0.0;
                motorSpeed_[1] = 0.0;
            }
        }
    }

    // Send target to motors.
    stepperMotors_.setSpeed(motorSpeed_);
}


void Robot::lowLevelThread()
{
    std::cout << "Low-level thread started." << std::endl;

    // Create metronome
    Metronome metronome(LOOP_PERIOD * 1e9);
    currentTime_ = 0;

    double lastTime = 0;

    while(TRUE)
    {
        // Wait for next tick.
        lastTime = currentTime_;
        metronome.wait();
        currentTime_ = metronome.getElapsedTime();

        // Update arduino data.
        uCData oldData = microcontrollerData_;
        microcontrollerData_ = uCListener_getData();

        // Update motor position.
        motorPosition_ = stepperMotors_.getPosition();

        double dt = currentTime_ - lastTime;

        // Estimate angle.
        double encoderIncrement[2];
        for(int i = 0; i < 2; i++)
            encoderIncrement[i] = (microcontrollerData_.encoderValues[i] - oldData.encoderValues[i]) * robotdimensions::encoderWheelRadius;

        double tanTheta = (encoderIncrement[RIGHT] - encoderIncrement[LEFT]) / robotdimensions::encoderWheelSpacing / 2.0;

        // Update position.
        RobotPosition currentPosition = currentPosition_.get();
        currentPosition.theta += atan(tanTheta);
        // Integrate angle estimation on X and Y.
        double linearIncrement = 1 / 2.0 * (encoderIncrement[LEFT] + encoderIncrement[RIGHT]);
        currentPosition.x += linearIncrement * std::cos(currentPosition.theta);
        // Minus sign: the frame is indirect.
        currentPosition.y -= linearIncrement * std::sin(currentPosition.theta);
        currentPosition_.set(currentPosition);

        updateTrajectoryFollowingTarget(dt);
        updateLog();
    }
}

void Robot::updateLog()
{
    logger_.setData(LOGGER_TIME, currentTime_);
    logger_.setData(LOGGER_COMMAND_VELOCITY_RIGHT, motorSpeed_[RIGHT]);
    logger_.setData(LOGGER_COMMAND_VELOCITY_LEFT, motorSpeed_[LEFT]);
    logger_.setData(LOGGER_MOTOR_POSITION_RIGHT, motorPosition_[RIGHT]);
    logger_.setData(LOGGER_MOTOR_POSITION_LEFT, motorPosition_[LEFT]);
    logger_.setData(LOGGER_ENCODER_RIGHT, microcontrollerData_.encoderValues[RIGHT]);
    logger_.setData(LOGGER_ENCODER_LEFT, microcontrollerData_.encoderValues[LEFT]);

    RobotPosition currentPosition = currentPosition_.get();
    logger_.setData(LOGGER_CURRENT_POSITION_X, currentPosition.x);
    logger_.setData(LOGGER_CURRENT_POSITION_Y, currentPosition.y);
    logger_.setData(LOGGER_CURRENT_POSITION_THETA, currentPosition.theta);

    logger_.setData(LOGGER_TARGET_POSITION_X, trajectoryPoint_.position.x);
    logger_.setData(LOGGER_TARGET_POSITION_Y, trajectoryPoint_.position.y);
    logger_.setData(LOGGER_TARGET_POSITION_THETA, trajectoryPoint_.position.theta);
    logger_.setData(LOGGER_TARGET_LINEAR_VELOCITY, trajectoryPoint_.linearVelocity);
    logger_.setData(LOGGER_TARGET_ANGULAR_VELOCITY, trajectoryPoint_.angularVelocity);

    logger_.setData(LOGGER_TRACKING_LONGITUDINAL_ERROR, trackingLongitudinalError_);
    logger_.setData(LOGGER_TRACKING_TRANSVERSE_ERROR, trackingTransverseError_);
    logger_.setData(LOGGER_TRACKING_ANGLE_ERROR, trackingAngleError_);
    logger_.writeLine();
}
