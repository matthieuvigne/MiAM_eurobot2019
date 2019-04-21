#include "Robot.h"

#include <iostream>

#include <algorithm>
#include <string>
#include <unistd.h>
#include <ctime>

void strategy(Robot *robot)
{
    std::cout << "Strategy thread started" << std::endl;


    // Servo along a trajectory.
    RobotPosition endPosition = robot->getCurrentPosition();
    std::cout << "Playing first trajectory" << std::endl;
    endPosition.x += 500;
    std::vector<std::shared_ptr<Trajectory>> traj;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot->getCurrentPosition(), endPosition);
    robot->setTrajectoryToFollow(traj);
    robot->waitForTrajectoryFinished();

    std::cout << "Strategy thread ended" << std::endl;
}
