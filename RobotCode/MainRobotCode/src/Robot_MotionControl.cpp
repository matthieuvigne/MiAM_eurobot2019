#include "Robot.h"
#include "MPC.h"

#define MAINROBOTCODE_USE_MPC false

// Control of the robot motion on the table.
// Trajectory list handling, trajectory following, obstacle avoidance.

bool Robot::followTrajectory(Trajectory *traj, double const& timeInTrajectory, double const & dt)
{
    
    if(MAINROBOTCODE_USE_MPC)
    {
        
        // NOTE: the MPC problem must be initialized when following a new traj
        // initialize_MPC_problem()
        
        // TODO here time is hardcoded
        if (timeInTrajectory >= traj->getDuration()) 
        {
             // Just stop the robot.
            motorSpeed_[0] = 0.0;
            motorSpeed_[1] = 0.0;
            return true;
        }
        
        // Current trajectory point
        miam::trajectory::TrajectoryPoint current_trajectory_point;
        
        // Position
        current_trajectory_point.position = currentPosition_.get();
        
        // Base speed
        BaseSpeed current_base_speed = getCurrentBaseSpeed();
        current_trajectory_point.linearVelocity = current_base_speed.linear;
        current_trajectory_point.angularVelocity = current_base_speed.angular;
        
        // Solve MPC
        miam::trajectory::TrajectoryPoint forward_traj_point = solve_MPC_problem(
            traj,
            current_trajectory_point,
            timeInTrajectory
        );
        
        // Convert to motor speed
        WheelSpeed ws = kinematics_.inverseKinematics(
            BaseSpeed(
                forward_traj_point.linearVelocity,
                forward_traj_point.angularVelocity
            )
        );
        
        motorSpeed_[RIGHT] = ws.right / robotdimensions::stepSize;
        motorSpeed_[LEFT] = ws.left / robotdimensions::stepSize;
        return false;
        
    } 
    else 
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

        // If trajectory has an angular velocity but no linear velocity, it's a point turn:
        // disable corresponding position servoing.
        if(std::abs(trajectoryPoint_.linearVelocity) > 0.1 || std::abs(trajectoryPoint_.angularVelocity) < 1e-4)
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

