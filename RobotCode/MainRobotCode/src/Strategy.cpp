#include <unistd.h>

#include "Robot.h"

void matchStrategy()
{
    std::cout << "Strategy thread started" << std::endl;

    //~ usleep(100000);
    //~ robot.moveRail(1);
    //~ robot.servos_.moveSuction(true);

    //~ robot.servos_.openTube(0);
    //~ robot.servos_.openTube(1);
    //~ robot.servos_.openTube(2);
    //~ robot.servos_.tapOpen();
    //~ robot.servos_.turnOnPump();
    //~ usleep(2000000);
    //~ robot.servos_.tapClose();
    //~ robot.servos_.closeTube(0);
    //~ robot.servos_.closeTube(2);
    //~ robot.servos_.openTube(1);

    //~ usleep(5000000);
    //~ robot.servos_.closeTube(1);
    //~ usleep(1000000);
    //~ robot.servos_.turnOffPump();

    //~ usleep(1000000);
    //~ robot.moveRail(0);
    //~ usleep(1000000);
    //~ robot.servos_.moveSuction(false);
    //~ usleep(1000000);
    //~ robot.servos_.openTube(0);
    //~ robot.servos_.openTube(1);
    //~ robot.servos_.openTube(2);
    //~ robot.servos_.tapOpen();

    // Servo along a trajectory.
    RobotPosition endPosition = robot.getCurrentPosition();
    std::cout << "Playing first trajectory" << std::endl;
    endPosition.x += 500;
    std::vector<std::shared_ptr<Trajectory>> traj;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), endPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    std::cout << "Playing second trajectory" << std::endl;
    RobotPosition startPosition = endPosition;
    endPosition.y += 500;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    std::cout << "Playing third trajectory" << std::endl;
    endPosition.theta = M_PI_2;
    startPosition = endPosition;
    endPosition.x -= 500;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    startPosition = endPosition;

    std::cout << "Playing fourth trajectory" << std::endl;
    endPosition.theta = M_PI;
    startPosition = endPosition;
    endPosition.y -= 500;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    usleep(200000);
    printf("done\n");
    while(true) ;;

    std::cout << "Strategy thread ended" << std::endl;
}
