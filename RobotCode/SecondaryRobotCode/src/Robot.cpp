#include "Robot.h"
#include <thread>
#include <unistd.h>

const int START_SWITCH = CAPE_DIGITAL[0];

// Update loop frequency
const double LOOP_PERIOD = 0.010;

Robot::Robot():
    AbstractRobot(),
    isIMUInit_(false),
    isScreenInit_(false),
    IRFrontLeft_(0),
    IRFrontRight_(0),
    IRBackLeft_(0),
    IRBackRight_(0),
    isFrontDetectionActive_(false),
    isBackDetectionActive_(false),
    hasDetectionStoppedRobot_(false),
    detectionStopTime_(0.0),
    startupStatus_(startupstatus::INIT),
    initMotorState_(0),
    nConsecutiveIRState_(0)
{
    kinematics_ = DrivetrainKinematics(robotdimensions::wheelRadius,
                                       robotdimensions::wheelSpacing,
                                       robotdimensions::encoderWheelRadius,
                                       robotdimensions::encoderWheelSpacing);
    // Update trajectory config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create logger.
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
    std::string filename = "/tmp/log" + std::string(timestamp) + ".csv";
    std::string headers = getHeaderStringList();
    // Log robot dimensions in header.
    std::string info = "wheelRadius:" + std::to_string(robotdimensions::wheelRadius) + \
                        "_wheelSpacing:" + std::to_string(robotdimensions::wheelSpacing) + \
                        "_stepSize:" + std::to_string(robotdimensions::stepSize);

    logger_ = Logger(filename, "Secondary robot", info, getHeaderStringList());

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 0;
    initialPosition.y = 0;
    initialPosition.theta = 0;
    currentPosition_.set(initialPosition);
    currentBaseSpeed_.linear = 0;
    currentBaseSpeed_.angular = 0;

    // Set PIDs.
    PIDLinear_ = miam::PID(controller::linearKp, controller::linearKd, controller::linearKi, 0.2);
    PIDAngular_ = miam::PID(controller::rotationKp, controller::rotationKd, controller::rotationKi, 0.15);
}

bool Robot::initSystem()
{
    bool allInitSuccessful = true;

    // Move the servos up ; no init required.
    robot.moveServos(false);

    if (!isScreenInit_)
    {
        isScreenInit_ = robot.screen_.init(&I2C_1);
        if (!isScreenInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with screen." << std::endl;
            #endif
            allInitSuccessful = false;
        }
        else
        {
            robot.screen_.setTextCentered("Initializing", 0);
            robot.screen_.setBacklight(true, true, true);
        }
    }
    if (!isIMUInit_)
    {
        isIMUInit_ = imu_initDefault(&imu_, &I2C_1, false);
        if (!isIMUInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with IMU." << std::endl;
            #endif
            allInitSuccessful = false;
            robot.screen_.setTextCentered("IMU init failed", 0);
            robot.screen_.setBacklight(true, false, false);
        }
    }
    if (!isStepperInit_)
    {
        // Motor config values - 42BYGHW810, 2.0A.
        const int MOTOR_KVAL_HOLD = 0x25;
        const int MOTOR_BEMF[4] = {0x36, 0x172E, 0xC, 0x33};

        // Compute max stepper motor speed.
        int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
        int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;
        // Initialize both motors.
        stepperMotors_ = miam::L6470("/dev/spidev1.0", 2);
        isStepperInit_ = stepperMotors_.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                             MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
        if (!isStepperInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with stepper motors." << std::endl;
            #endif
            allInitSuccessful = false;
            robot.screen_.setTextCentered("Motor init failed", 0);
            robot.screen_.setBacklight(true, false, false);
        }
        else
            stepperMotors_.highZ();
    }
    if(allInitSuccessful)
        gpio_digitalWrite(CAPE_LED[1], 0);
    else
        gpio_digitalWrite(CAPE_LED[1], 1);
    return allInitSuccessful;
}


bool Robot::setupBeforeMatchStart()
{
    // Once the match has started, nothing remains to be done.
    if (hasMatchStarted_)
        return true;
    // Action depends on current startup status
    if (startupStatus_ == startupstatus::INIT)
    {
        // Try to initialize system.
        bool isInit = initSystem();
        if (isInit)
        {
            startupStatus_ = startupstatus::WAITING_FOR_CABLE;
            robot.screen_.setTextCentered("Waiting for", 0);
            robot.screen_.setTextCentered("start switch", 1);
            robot.screen_.setBacklight(true, true, true);
        }
    }
    else if (startupStatus_ == startupstatus::WAITING_FOR_CABLE)
    {
        // Wait for cable to be plugged in.
        if (gpio_digitalRead(START_SWITCH) == 0)
        {
            // Store plug time in matchStartTime_ to prevent false start due to switch bounce.
            matchStartTime_ = currentTime_;
            startupStatus_ = startupstatus::PLAYING_LEFT;
            isPlayingRightSide_ = false;
            robot.screen_.setTextCentered("Choose side", 0);
            robot.screen_.setTextCentered("     YELLOW    >", 1);
            robot.screen_.setBacklight(true, true, true);
        }
    }
    else
    {
        // Switch side based on button press.
        if (startupStatus_ == startupstatus::PLAYING_RIGHT && robot.screen_.isButtonPressed(LCD_BUTTON_LEFT))
        {
            startupStatus_ = startupstatus::PLAYING_LEFT;
            isPlayingRightSide_ = false;
            robot.screen_.setTextCentered("     YELLOW    >", 1);
            robot.screen_.setBacklight(true, true, true);
        }
        else if (startupStatus_ == startupstatus::PLAYING_LEFT && robot.screen_.isButtonPressed(LCD_BUTTON_RIGHT))
        {
            startupStatus_ = startupstatus::PLAYING_RIGHT;
            isPlayingRightSide_ = true;
            robot.screen_.setTextCentered("<    PURPLE     ", 1);
            robot.screen_.setBacklight(true, true, true);
        }

        // Use down button to lock / unlock the motors.
        if (robot.screen_.isButtonPressed(LCD_BUTTON_DOWN))
        {
            // Only modify state if the button was just pressed.
            if (initMotorState_ < 2)
            {
                if (initMotorState_ == 0)
                    stepperMotors_.hardStop();
                else
                    stepperMotors_.highZ();
                initMotorState_ = (initMotorState_ + 1) % 2 + 2;
            }
        }
        else
        {
            // Button release: change state to be less that two.
            initMotorState_ = initMotorState_ % 2;
        }

        // If start button is pressed, return true to end startup.
        if (currentTime_ - matchStartTime_ > 0.5 && gpio_digitalRead(START_SWITCH) == 1)
        {
            robot.screen_.clear();
            return true;
        }
    }

    return false;
}


void Robot::lowLevelLoop()
{
    std::cout << "Low-level loop started." << std::endl;

    // Create metronome
    Metronome metronome(LOOP_PERIOD * 1e9);
    currentTime_ = 0;

    double lastTime = 0;

    std::vector<double> oldMotorAngle;
    oldMotorAngle.push_back(0.0);
    oldMotorAngle.push_back(0.0);

    std::thread strategyThread;

    int nIter = 0;
    bool heartbeatLed = true;
    // Loop until start of the match, then for 100 seconds after the start of the match.
    while(!hasMatchStarted_ || (currentTime_ < 100.0 + matchStartTime_))
    {
        // Wait for next tick.
        lastTime = currentTime_;
        metronome.wait();
        currentTime_ = metronome.getElapsedTime();
        double dt = currentTime_ - lastTime;

        // Heartbeat.
        nIter ++;
        if (nIter % 50 == 0)
        {
            heartbeatLed = !heartbeatLed;
            gpio_digitalWrite(CAPE_LED[0], heartbeatLed);
        }
        // If match hasn't started, look at switch value to see if it has.
        if (!hasMatchStarted_)
        {
            hasMatchStarted_ = setupBeforeMatchStart();
            if (hasMatchStarted_)
            {
                matchStartTime_ = currentTime_;
                // Start strategy thread.
                strategyThread = std::thread(&matchStrategy);
                strategyThread.detach();
            }
        }

        // Update motor position.
        motorPosition_ = stepperMotors_.getPosition();

        std::vector<double> motorAngle;
        motorAngle.push_back(robotdimensions::stepSize * motorPosition_[0]);
        motorAngle.push_back(robotdimensions::stepSize * motorPosition_[1]);

        // Update position through integration of motor measurments.
        WheelSpeed encoderIncrement;
        encoderIncrement.right = motorAngle[RIGHT] - oldMotorAngle[RIGHT];
        encoderIncrement.left = motorAngle[LEFT] - oldMotorAngle[LEFT];
        oldMotorAngle = motorAngle;

        // If playing right side: invert right/left encoders, with minus sign because both motors are opposite of each other.
        if (isPlayingRightSide_)
        {
            double temp = encoderIncrement.right;
            encoderIncrement.right = encoderIncrement.left;
            encoderIncrement.left = temp;
        }

        RobotPosition currentPosition = currentPosition_.get();
        kinematics_.integratePosition(encoderIncrement, currentPosition);
        currentPosition_.set(currentPosition);

        // Perform trajectory tracking.
        updateTrajectoryFollowingTarget(dt);

        // Get current encoder speeds (in rad/s)
        WheelSpeed instantWheelSpeedEncoder;
        instantWheelSpeedEncoder.right = encoderIncrement.right / dt;
        instantWheelSpeedEncoder.left = encoderIncrement.left / dt;

        // Get base speed
        currentBaseSpeed_ = kinematics_.forwardKinematics(instantWheelSpeedEncoder, true);

        // Update log.
        updateLog();
    }
    // End of the match.
    std::cout << "Match end" << std::endl;
    robot.screen_.setBacklight(false, false, false);
    pthread_cancel(strategyThread.native_handle());
    stopMotors();
}

void Robot::updateLog()
{
    logger_.setData(LOGGER_TIME, currentTime_);
    logger_.setData(LOGGER_COMMAND_VELOCITY_RIGHT, motorSpeed_[RIGHT]);
    logger_.setData(LOGGER_COMMAND_VELOCITY_LEFT, motorSpeed_[LEFT]);
    logger_.setData(LOGGER_MOTOR_POSITION_RIGHT, motorPosition_[RIGHT]);
    logger_.setData(LOGGER_MOTOR_POSITION_LEFT, motorPosition_[LEFT]);
    logger_.setData(LOGGER_ENCODER_RIGHT, motorPosition_[RIGHT]);
    logger_.setData(LOGGER_ENCODER_LEFT, motorPosition_[LEFT]);

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
    logger_.setData(LOGGER_I_R_FRONT_LEFT, IRFrontLeft_);
    logger_.setData(LOGGER_I_R_FRONT_RIGHT, IRFrontRight_);
    logger_.setData(LOGGER_I_R_BACK_LEFT, IRBackLeft_);
    logger_.setData(LOGGER_I_R_BACK_RIGHT, IRBackRight_);
    logger_.setData(LOGGER_FRONT_DETECTION, isFrontDetectionActive_);
    logger_.setData(LOGGER_BACK_DETECTION, isBackDetectionActive_);

    logger_.setData(LOGGER_LINEAR_P_I_D_CORRECTION, PIDLinear_.getCorrection());
    logger_.setData(LOGGER_ANGULAR_P_I_D_CORRECTION, PIDAngular_.getCorrection());
    logger_.writeLine();
}


void Robot::moveServos(bool down)
{
    if(down)
    {
        gpio_servoPWM(0, 1155); // Right
        gpio_servoPWM(1, 1925); // Left
    }
    else
    {
        gpio_servoPWM(0, 2200);
        gpio_servoPWM(1, 900);
    }
}
