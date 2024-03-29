#include "Robot.h"

// Control of the robot motion on the table.
// Trajectory list handling, trajectory following, obstacle avoidance.

// Obstacle avoidance parameters
double const IR_START_TIMEOUT = 5.0; // Disable IR detection for the first n seconds of the match, to be sure to leave the starting zone safely.

double const FRONT_DETECTION_DISTANCE = 750;    // Distance between center of the robot and detected obstacle.
double const BACK_DETECTION_DISTANCE = 750;

int const N_VIEWS_FOR_STOP = 2; // Number of consecutive views to stop the robot.
int const N_MISS_FOR_START = 5; // Number of consecutive non-detection to restart robot.

bool Robot::handleDetection()
{
    // Read IR sensors.
    IRFrontLeft_ = gpio_analogRead(CAPE_ANALOG[1]);
    IRFrontRight_ = gpio_analogRead(CAPE_ANALOG[4]);
    IRBackLeft_ = gpio_analogRead(CAPE_ANALOG[2]);
    IRBackRight_ = gpio_analogRead(CAPE_ANALOG[3]);

    // Determine if a robot is visible in front or behind the robot.
    isFrontDetectionActive_ = (IRFrontLeft_ > robot.frontDetectionThreshold_ || IRFrontRight_ > robot.frontDetectionThreshold_);
    isBackDetectionActive_ = (IRBackLeft_ > robot.backDetectionThreshold_ || IRBackRight_ > robot.backDetectionThreshold_);

    // Set LED status
    gpio_digitalWrite(CAPE_LED[1], isFrontDetectionActive_ || isBackDetectionActive_);

    bool shouldRobotStop = false;

    bool isGoingForward = true;
    // Determine if there is an obstacle in the direction we are going,
    if (trajectoryPoint_.linearVelocity > 0.01 && isFrontDetectionActive_)
        isGoingForward = true;
    else if (trajectoryPoint_.linearVelocity < -0.01 && isBackDetectionActive_)
        isGoingForward = false;
    else
        return false; // If we have no forward or backward desired velocity (i.e. point turn), no need to do any detection.

    // Find obstacle position.
    RobotPosition obstaclePosition;
    if (isGoingForward)
        obstaclePosition.x = FRONT_DETECTION_DISTANCE;
    else
        obstaclePosition.x -= BACK_DETECTION_DISTANCE;

    RobotPosition robotPosition = getCurrentPosition();
    obstaclePosition = obstaclePosition.rotate(robotPosition.theta) + robotPosition;

    // If the obstacle is outside of the playing field, ignore it.
    if (obstaclePosition.x > 0 && obstaclePosition.x < 3000 && obstaclePosition.y > 0 && obstaclePosition.y < 2000)
        shouldRobotStop = true;

    if (!hasMatchStarted_)
        return false;
    if (currentTime_ - matchStartTime_ < IR_START_TIMEOUT)
        return false;
    //~ return shouldRobotStop;
    return false;
}


bool Robot::followTrajectory(Trajectory *traj, double const& timeInTrajectory, double const & dt)
{
    // Get current trajectory state.
    trajectoryPoint_ = traj->getCurrentPoint(timeInTrajectory);

    // Compute targets for rotation and translation motors.
    BaseSpeed targetSpeed;

    // Feedforward.
    targetSpeed.linear = trajectoryPoint_.linearVelocity;
    targetSpeed.angular = trajectoryPoint_.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_.get();
    RobotPosition error = currentPosition - trajectoryPoint_.position;

    // Rotate by -theta to express the error in the tangent frame.
    RobotPosition rotatedError = error.rotate(-trajectoryPoint_.position.theta);

    trackingLongitudinalError_ = rotatedError.x;
    trackingTransverseError_ = rotatedError.y;

    // Change sign if going backward.
    if(trajectoryPoint_.linearVelocity < 0)
        trackingTransverseError_ = - trackingTransverseError_;
    trackingAngleError_ = miam::trajectory::moduloTwoPi(currentPosition.theta - trajectoryPoint_.position.theta);

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

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 || std::abs(trajectoryPoint_.angularVelocity) < 1e-4)
        targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError_, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError_;
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 * robotdimensions::maxWheelSpeed)
        angularPIDError += controller::transverseKp * trajectoryPoint_.linearVelocity / robotdimensions::maxWheelSpeed * trackingTransverseError_;
    targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

    // Invert velocity if playing on right side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // Convert from base velocity to motor wheel velocity.
    WheelSpeed wheelSpeed = kinematics_.inverseKinematics(targetSpeed);
    // Convert to motor unit.
    motorSpeed_[RIGHT] = wheelSpeed.right / robotdimensions::stepSize;
    motorSpeed_[LEFT] = wheelSpeed.left / robotdimensions::stepSize;

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
        trajectoryPoint_ = currentTrajectories_[0]->getCurrentPoint(0.0);
        std::cout << "Recieved new trajectory" << std::endl;
    }

    // Hande detection.
    bool iRStatus = handleDetection();
    // Reset counter if not in the right state
    if ((iRStatus && nConsecutiveIRState_ < 0) || (!iRStatus && nConsecutiveIRState_ > 0))
        nConsecutiveIRState_ = 0;
    nConsecutiveIRState_ += (iRStatus ? 1 : -1);

    bool shouldRobotStop = false;
    // If enough successive detection, stop the robot.
     if (nConsecutiveIRState_ >= N_VIEWS_FOR_STOP && !hasDetectionStoppedRobot_)
    {
        hasDetectionStoppedRobot_ = true;
        detectionStopTime_ = currentTime_;
        shouldRobotStop = true;
    }

    // If detection has stopped robot, look if we have enough miss to restart
    if (hasDetectionStoppedRobot_)
        if (nConsecutiveIRState_ > - N_MISS_FOR_START)
            shouldRobotStop = true;

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

        // Stop for at least a certain time, to remove jitter.
        if (shouldRobotStop || currentTime_ - detectionStopTime_ < 0.5)
        {
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            stepperMotors_.hardStop();
            // TODO: add timeout on detection ?
        }
        else
        {
            if (hasDetectionStoppedRobot_)
            {
                hasDetectionStoppedRobot_ = false;
                // Robot was previously stopped but there is no longer any obstacle.
                // We replanify the trajectory and restart following.
                traj->replanify(detectionStopTime_ - trajectoryStartTime_);
                trajectoryStartTime_ = currentTime_;
            }

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
    }
    stepperMotors_.getError();

    // Send target to motors.
    if (hasMatchStarted_ && !shouldRobotStop)
        stepperMotors_.setSpeed(motorSpeed_);
}

