#include "Robot.h"

#include <iostream>

#include <algorithm>
#include <string>
#include <unistd.h>
#include <ctime>

using miam::trajectory::rotationside;
using miam::trajectory::Trajectory;
using miam::trajectory::ArcCircle;
using miam::trajectory::PointTurn;

// Robot dimension.
double const CHASSIS_FRONT = 46.0;
double const CHASSIS_BACK = 75.0;
double const CHASSIS_WIDTH = 169.0;

void matchStrategy()
{
    std::cout << "Strategy thread started" << std::endl;

    robot.moveServos(true);
    usleep(800000);

    // Set initial position
    RobotPosition targetPosition;
    targetPosition.x = CHASSIS_WIDTH + 25.0;
    targetPosition.y = 1700 - CHASSIS_FRONT - 100.0;
    targetPosition.theta = M_PI_2;
    robot.resetPosition(targetPosition, true, true, true);

    std::vector<std::shared_ptr<miam::trajectory::Trajectory>> traj;
    std::vector<RobotPosition> positions;

    // Go get chaos zone, following a curved trajectory.
    positions.push_back(targetPosition);
    targetPosition.y = 2000 - CHASSIS_WIDTH - 50;
    positions.push_back(targetPosition);
    targetPosition.x = 1020;
    positions.push_back(targetPosition);
    targetPosition.y = 1020;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.4);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Circle inside the chaos zone to grab all the atoms.
    std::shared_ptr<ArcCircle> circle(new ArcCircle(robot.getCurrentPosition(), 200.0, rotationside::RIGHT, - 1.2));
    traj.clear();
    traj.push_back(circle);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Go to the corner of the playing field, then up the ramp.
    positions.clear();
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    // Compute coordinate to generate no rotation and reach a point with x coordinate at 200.
    targetPosition.x = 200;
    double dx = targetPosition.x - robot.getCurrentPosition().x;
    targetPosition.y += dx * std::tan(targetPosition.theta);
    positions.push_back(targetPosition);
    targetPosition.y = 400;
    positions.push_back(targetPosition);
    targetPosition.y = 200;
    positions.push_back(targetPosition);
    targetPosition.x = 300;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.4);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();


    // Go hit back wall to reset position.
    targetPosition.x = CHASSIS_BACK - 10.0;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    targetPosition.x = CHASSIS_BACK;
    targetPosition.y = robot.getCurrentPosition().y;
    targetPosition.theta = 0.0;
    robot.resetPosition(targetPosition, true, false, true);

    // Go push the atoms at the top of the ramp.
    targetPosition.x = 1228 - CHASSIS_FRONT - 30;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    std::cout << "Strategy thread ended" << std::endl;
}
