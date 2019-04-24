#include "Robot.h"

// Control of the robot motion on the table.
// Trajectory list handling, trajectory following, obstacle avoidance.

// Obstacle avoidance parameters
int const FRONT_THRESHOLD = 500; // Any value below the threshold corresponds to a detection.
int const BACK_THRESHOLD = 500; // Any value below the threshold corresponds to a detection.
double const IR_START_TIMEOUT = 5.0; // Disable IR detection for the first n seconds of the match, to be sure to leave the starting zone safely.

double const FRONT_DETECTION_DISTANCE = 500;    // Distance between front of the robot and detected obstacle.
double const BACK_DETECTION_DISTANCE = 500;


bool Robot::handleDetection()
{
    // Read IR sensors.
    IRFrontLeft_ = gpio_analogRead(CAPE_ANALOG[1]);
    IRFrontRight_ = gpio_analogRead(CAPE_ANALOG[4]);
    IRBackLeft_ = gpio_analogRead(CAPE_ANALOG[2]);
    IRBackRight_ = gpio_analogRead(CAPE_ANALOG[3]);

    // Determine if a robot is visible in front or behind the robot.
    isFrontDetectionActive_ = (IRFrontLeft_ > FRONT_THRESHOLD || IRFrontRight_ > FRONT_THRESHOLD);
    isBackDetectionActive_ = (IRBackLeft_ > BACK_THRESHOLD || IRBackRight_ > BACK_THRESHOLD);

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
    return shouldRobotStop;
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
    targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError_, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    double angularPIDError = trackingAngleError_;
    if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 * robotdimensions::maxWheelSpeed)
        angularPIDError += controller::transverseKp * trajectoryPoint_.linearVelocity / robotdimensions::maxWheelSpeed * trackingTransverseError_;
    targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

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
    bool shouldRobotStop = handleDetection();
    std::cout << shouldRobotStop << std::endl;

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

        // Handle detection.
         if (shouldRobotStop && !hasDetectionStoppedRobot_)
        {
            hasDetectionStoppedRobot_ = true;
            detectionStopTime_ = currentTime_;
        }

        // Stop for at least a certain time, to remove jitter.
        if (shouldRobotStop || currentTime_ - detectionStopTime_ < 0.5)
        {
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            stopMotors();
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

    // Send target to motors.
    stepperMotors_.setSpeed(motorSpeed_);
}

