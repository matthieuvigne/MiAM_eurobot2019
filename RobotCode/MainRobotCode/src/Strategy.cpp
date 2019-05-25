#include <unistd.h>

#include "Robot.h"

// Robot dimension.
double const CHASSIS_FRONT = 150.0;
double const CHASSIS_BACK = 150.0;
double const CHASSIS_WIDTH = 150.0;

using namespace miam::trajectory;

// Aquire the atoms, assuming the robot is 5cm in front of them.
void getAtoms(double moveAmount = 50, bool moveSuction=true)
{
    // Open tap, turn on pump, open all suction caps.
    if (moveSuction)
        robot.moveRail(0.30);

    robot.servos_.turnOnPump();
    robot.servos_.tapOpen();
    robot.servos_.moveSuction(true);
    for(int i = 0; i < 3; i++)
        robot.servos_.openTube(i);
    usleep(200000);

    // Move
    RobotPosition targetPosition = robot.getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, moveAmount);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Close tap.
    robot.servos_.tapClose();
    usleep(500000);
    for(int i = 0; i < 3; i++)
        robot.servos_.closeTube(i);
    usleep(1000000);
    if (moveSuction)
        robot.moveRail(0.6);

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -moveAmount);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    robot.servos_.turnOffPump();
}

void matchStrategy()
{
    std::cout << "Strategy thread started." << std::endl;

    // Real game strategy.
    RobotPosition targetPosition;
    targetPosition.x = CHASSIS_WIDTH + 75;
    targetPosition.y = 1100 + CHASSIS_FRONT + 30;
    targetPosition.theta = -M_PI_2;

    robot.resetPosition(targetPosition, true, true, true);
    //~ robot.moveRail(0.25);

    // Update config.
    // Increase wheel spacing to slow down rotations.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    TrajectoryVector traj;
    std::vector<RobotPosition> positions;

    //**********************************************************
    // Go get first atoms.
    //**********************************************************
    //~ targetPosition.y = 150;
    targetPosition.y = CHASSIS_FRONT + 50;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    getAtoms();
    robot.moveRail(0.05);
    robot.servos_.moveSuction(false);

    // Move back on base, drop all three atoms.
    targetPosition.y = 1710;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);
    robot.setTrajectoryToFollow(traj);
    // Atom drop
    while (!robot.isTrajectoryFinished())
    {
        targetPosition = robot.getCurrentPosition();
        if (targetPosition.y > 950)
        {
            robot.servos_.tapOpen();
            robot.servos_.openTube(2);
        }
        if (targetPosition.y > 1300)
            robot.servos_.openTube(1);
        if (targetPosition.y > 1600)
            robot.servos_.openTube(0);
        usleep(20000);
    }
    robot.updateScore(18);
    robot.moveRail(0.3);
    robot.servos_.moveSuction(true);


    //**********************************************************
    // Get second set of atoms.
    //**********************************************************

    // Go get left set of three atoms, following a curved trajectory.
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    targetPosition.x += 220;
    targetPosition.y -=40;
    positions.push_back(targetPosition);
    targetPosition.y = 800;
    positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.y = 510 + CHASSIS_FRONT;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Grab atoms.
    getAtoms();
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    robot.moveRail(0.05);
    robot.servos_.moveSuction(false);

    // Go drop green atom.
    targetPosition = robot.getCurrentPosition();
    targetPosition.y = 1250;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    targetPosition.x = 450;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    robot.servos_.tapOpen();
    robot.servos_.openTube(1);
    usleep(1000000);
    robot.servos_.tapClose();
    robot.servos_.closeTube(1);
    robot.updateScore(6); // Drop atom in the red zone.

    // Go drop red atom.
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    positions.clear();
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1550;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    robot.servos_.tapOpen();
    robot.servos_.openTube(0);
    robot.servos_.openTube(2);
    usleep(1000000);

    robot.updateScore(12);
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    //**********************************************************
    // Drop the blue atom in the particle accelerator.
    //**********************************************************
    targetPosition = robot.getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 1300;
    positions.push_back(targetPosition);
    targetPosition.x = 1600;
    targetPosition.y = 2000 - CHASSIS_WIDTH - 130;
    positions.push_back(targetPosition);
    targetPosition.x = 1620;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.1);
    robot.setTrajectoryToFollow(traj);
    usleep(100000);
    while (!robot.isTrajectoryFinished())
    {
        targetPosition = robot.getCurrentPosition();
        if (targetPosition.x > 1250)
        {
            robot.servos_.unfoldArms(robot.isPlayingRightSide());
            break;
        }
    }
    robot.waitForTrajectoryFinished();
    robot.servos_.raiseArms(robot.isPlayingRightSide());
    robot.updateScore(20);

    //**********************************************************
    // Get the goldium atom.
    //**********************************************************
    robot.moveRail(1.0);
    robot.servos_.moveSuction(true);
    targetPosition = robot.getCurrentPosition();
    targetPosition.x = 3000 - 770;
    targetPosition.y = 1650;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    targetPosition.y = 2000 - CHASSIS_FRONT - 100;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    robot.servos_.tapClose();
    robot.servos_.closeTube(0);
    robot.servos_.openTube(1);
    robot.servos_.closeTube(2);
    robot.servos_.turnOnPump();
    usleep(500000);
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 40);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    usleep(500000);
    robot.servos_.closeTube(2);
    robot.servos_.tapClose();
    usleep(500000);
    robot.servos_.turnOffPump();
    usleep(500000);
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    robot.updateScore(20);
    robot.servos_.openTube(2);


    //**********************************************************
    // Push things from the chaos zone
    //**********************************************************
    targetPosition = robot.getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 1200;
    targetPosition.y = 950;
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    robot.updateScore(9);

    std::cout << "Strategy thread ended" << std::endl;
}
