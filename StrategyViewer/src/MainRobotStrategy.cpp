#include "ViewerRobot.h"
#include <MiAMEurobot/trajectory/Utilities.h>
#include <MiAMEurobot/trajectory/ArcCircle.h>

using namespace miam::trajectory;

// Define robot constants for trajectory generation.
namespace robotdimensions
{
    double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
    double const maxWheelSpeed = 500; ///< Maximum wheel speed, in mm/s.
    double const maxWheelAcceleration = 700; ///< Maximum wheel acceleration, in mm/s^2.
}

// Robot dimension.
double const CHASSIS_FRONT = 150.0;
double const CHASSIS_BACK = 150.0;
double const CHASSIS_WIDTH = 150.0;


TrajectoryVector fallbackToGreenZone(ViewerRobot & robot)
{
    RobotPosition targetPosition = robot.getPosition();

    // Turn around and move left.
    RobotPosition endPosition = targetPosition;
    endPosition.x -= 200;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStaightLineToPoint(targetPosition, endPosition);
    targetPosition = traj.getEndPoint().position;

    std::vector<RobotPosition> positions;
    positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = 1250;
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    traj = traj + miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
    return traj;

}


void mainRobotStrategy(ViewerRobot &robot)
{
    robot.trajectory_.clear();
    std::cout << "Computing main robot strategy, obstacle at " << robot.obstacleX_ << " " << robot.obstacleY_ << std::endl;
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
    TrajectoryVector traj;
    std::vector<RobotPosition> positions;

    robot.score_ = 40; // Experiment points.

    // Go get first atoms.
    targetPosition.y = 150;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition);
    robot.followTrajectory(traj);

    // Move back on base, drop all three atoms.
    targetPosition.y = 1800;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0.0, true);
    robot.followTrajectory(traj);

    robot.score_ += 18;

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
    robot.score_ += 4 * 6 + 1; // Drop 5 atoms in the red zone.

    targetPosition = robot.getPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
    robot.followTrajectory(traj);

    // Next phase: go reach the goldium.
    // Perform a choice based on oponent robot position.
    if (robot.obstacleY_ < 1500 || robot.obstacleX_ > 1500)
    {
        // Shortest way: go along the limit.
        targetPosition = robot.getPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x += 300;
        targetPosition.y = 2000 - CHASSIS_WIDTH - 90;
        positions.push_back(targetPosition);
        targetPosition.x = 2000;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
        targetPosition = traj.getEndPoint().position;
        RobotPosition endPosition = targetPosition;
        endPosition.y = 2000 - CHASSIS_FRONT - 44;
        traj = traj +  miam::trajectory::computeTrajectoryStaightLineToPoint(targetPosition, endPosition);
    }
    else
    {
        // Go below by the middle of the field.
        targetPosition = robot.getPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x += 80;
        targetPosition.y = 1400;
        positions.push_back(targetPosition);
        targetPosition.x = 2000;
        positions.push_back(targetPosition);
        targetPosition.y = 2000 - CHASSIS_FRONT - 44;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
    }

    bool phaseSucceeded = robot.followTrajectory(traj);

    if (phaseSucceeded)
    {
        // Handle goldium.
        // Drop current atom.
        robot.score_ += 20;
        targetPosition = robot.getPosition();
        // Go get goldium.
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
        targetPosition = traj.getEndPoint().position;
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 2500 - 290;
        positions.push_back(targetPosition);
        targetPosition.y = 2000 - CHASSIS_FRONT - 44;
        positions.push_back(targetPosition);
        traj = traj + miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
        phaseSucceeded = robot.followTrajectory(traj);
    }
    if (phaseSucceeded)
    {
        // Grab goldium
        robot.score_ += 20;

        // Go put it down in the scale.
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
        robot.followTrajectory(traj);
        targetPosition = robot.getPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 1300;
        targetPosition.y = 1200;
        positions.push_back(targetPosition);
        targetPosition.y = 450 + CHASSIS_FRONT;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
        phaseSucceeded = robot.followTrajectory(traj);
    }
    if (phaseSucceeded)
    {
        // Drop goldium
        robot.score_ += 24;
        targetPosition = robot.getPosition();
        // Go grab last three atoms.
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
        robot.followTrajectory(traj);
        targetPosition = robot.getPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 900;
        positions.push_back(targetPosition);
        targetPosition.y = 450 + CHASSIS_FRONT;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
        robot.followTrajectory(traj);
    }
    else
    {
        // Fallback strategy : go put down green atom.
        // Keep trying forever
        bool success = false;
        while (!success)
        {
            traj = fallbackToGreenZone(robot);
            success = robot.followTrajectory(traj);
        }
        // Put down current atom : goldium or greenium, it's the same.
        robot.score_ += 6;
        // Go grab the three atoms on the wall.
        targetPosition = robot.getPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
        robot.followTrajectory(traj);
        targetPosition = traj.getEndPoint().position;
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 900;
        targetPosition.y = 800;
        positions.push_back(targetPosition);
        targetPosition.y = 450 + CHASSIS_FRONT;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.4);
        robot.followTrajectory(traj);
    }

    // Go put down the three atoms.

    // Blue
    targetPosition = robot.getPosition();
    targetPosition.y = 950;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition, 0, true);
    robot.followTrajectory(traj);
    targetPosition.x = 450;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getPosition(), targetPosition);
    robot.followTrajectory(traj);
    robot.score_ += 6;

    // Green
    targetPosition = robot.getPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.followTrajectory(traj);
    positions.clear();
    targetPosition = robot.getPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1250;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.4);
    robot.followTrajectory(traj);
    robot.score_ += 6;

    // Red
    targetPosition = robot.getPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.followTrajectory(traj);
    positions.clear();
    targetPosition = robot.getPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1550;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.4);
    robot.followTrajectory(traj);
    robot.score_ += 6;



}

