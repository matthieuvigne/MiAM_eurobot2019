#include "ViewerRobot.h"
#include <MiAMEurobot/trajectory/Utilities.h>
#include <MiAMEurobot/trajectory/ArcCircle.h>

using miam::trajectory::rotationside;
using miam::trajectory::Trajectory;
using miam::trajectory::ArcCircle;

// Define robot constants for trajectory generation.
namespace robotdimensions
{
    double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
    double const maxWheelSpeed = 500; ///< Maximum wheel speed, in mm/s.
    double const maxWheelAcceleration = 700; ///< Maximum wheel acceleration, in mm/s^2.
}

void mainRobotStrategy(ViewerRobot &robot)
{
    RobotPosition targetPosition;
    targetPosition.x = 200;
    targetPosition.y = 1150;
    targetPosition.theta = -G_PI_2;

    // Update config.
    // Increase wheel spacing to slow down rotations.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeed,
                                                    robotdimensions::maxWheelAcceleration,
                                                    robotdimensions::wheelSpacing);
    robot.setPosition(targetPosition);

    RobotPosition resetPosition;
    std::vector<std::shared_ptr<miam::trajectory::Trajectory>> traj;

    std::vector<RobotPosition> positions;

    // Go get first atoms.
    targetPosition.y = 150;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition);
    robot.followTrajectory(traj);
    // Position reset

    // Move back on base, drop all three atoms.
    targetPosition.y = 1800;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0, true);
    robot.followTrajectory(traj);

    // Go get left set of three atoms, following a curved trajectory.
    targetPosition = robot.getPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    targetPosition.x += 200;
    targetPosition.y -=40;
    positions.push_back(targetPosition);
    targetPosition.y = 800;
    positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.y = 600;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
    robot.followTrajectory(traj);

    // Go back, and push the atoms to the end of the red zone.
    targetPosition.y += 100;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0, true);
    robot.followTrajectory(traj);
    targetPosition.y = 1400;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0);
    robot.followTrajectory(traj);

    // Turn to put the atoms in the red zone.
    std::shared_ptr<ArcCircle> circle(new ArcCircle(robot.getPosition(), 150.0, rotationside::LEFT, M_PI_2));
    traj.clear();
    traj.push_back(circle);
    robot.followTrajectory(traj);

    targetPosition = robot.getPosition();
    targetPosition.x -= 50;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0);
    robot.followTrajectory(traj);

    // Drop the red atoms.

    // Now move back to drop the green atoms.
    targetPosition.x += 250;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0, true);
    robot.followTrajectory(traj);

    positions.clear();
    positions.push_back(robot.getPosition());
    targetPosition.y = 1250;
    positions.push_back(targetPosition);
    targetPosition.x = 400;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.4);
    robot.followTrajectory(traj);

}

