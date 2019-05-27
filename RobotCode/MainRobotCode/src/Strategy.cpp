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

    // Update config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    bool wasMoveSuccessful = true;

    // Set initial position
    targetPosition.x = CHASSIS_WIDTH + 75;
    targetPosition.y = 1100 + CHASSIS_FRONT + 30;
    targetPosition.theta = -M_PI_2;
    robot.resetPosition(targetPosition, true, true, true);

    //**********************************************************
    // Go get first atoms.
    //**********************************************************
    targetPosition.y = CHASSIS_FRONT + 50;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();

    getAtoms();
    robot.moveRail(0.05);
    robot.servos_.moveSuction(false);

    // Move back on base, drop all three atoms.
    targetPosition.y = 1710;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);
    robot.setTrajectoryToFollow(traj);
    // Atom drop
    int suctionOrder[3] = {2, 1, 0};
    if (robot.isPlayingRightSide())
    {
        suctionOrder[0] = 0;
        suctionOrder[2] = 2;
    }
    while (!robot.isTrajectoryFinished())
    {
        targetPosition = robot.getCurrentPosition();
        if (targetPosition.y > 950)
        {
            robot.servos_.tapOpen();
            robot.servos_.openTube(suctionOrder[0]);
        }
        if (targetPosition.y > 1300)
            robot.servos_.openTube(suctionOrder[1]);
        if (targetPosition.y > 1600)
            robot.servos_.openTube(suctionOrder[2]);
        usleep(20000);
    }
    if (wasMoveSuccessful)
        robot.updateScore(18); // 3 atoms with the right color inside the zone.
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
    targetPosition.y -= 40;
    positions.push_back(targetPosition);
    targetPosition.y = 800;
    positions.push_back(targetPosition);
    targetPosition.x = 600;
    targetPosition.y = 700;
    positions.push_back(targetPosition);
    targetPosition.y = 520 + CHASSIS_FRONT;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();

    // Grab atoms.
    bool hasAtoms = false;
    if (wasMoveSuccessful)
    {
        getAtoms();
        hasAtoms = true;
    }
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    robot.moveRail(0.05);
    robot.servos_.moveSuction(false);

    // Go drop atoms.
    // Drop everything in the red zone, pushing the tree atoms on the field.
    positions.clear();
    targetPosition = robot.getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1550;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();

    robot.servos_.tapOpen();
    robot.servos_.openTube(0);
    robot.servos_.openTube(1);
    robot.servos_.openTube(2);
    if (wasMoveSuccessful)
        robot.updateScore(13); // Three atoms pushed in the zone, two of the right color.
    if (hasAtoms)
        robot.updateScore(13); // Three atoms dropped in the zone, two of the right color.

    // Move back
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();


    //**********************************************************
    // Push things from the chaos zone
    //**********************************************************
    targetPosition = robot.getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 1350;
    targetPosition.y = 1300;
    positions.push_back(targetPosition);
    targetPosition.y = 950;
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();
    if (wasMoveSuccessful)
        robot.updateScore(9);   // 4 atoms pushed, one of the right color.

    //**********************************************************
    // Get the last three set of atoms.
    //**********************************************************
    if (wasMoveSuccessful)
    {
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
    }
    targetPosition = robot.getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 900;
    targetPosition.y = 950;
    positions.push_back(targetPosition);
    targetPosition.y = 520 + CHASSIS_FRONT;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.05);
    robot.setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot.waitForTrajectoryFinished();

    // Grab atoms.
    if (wasMoveSuccessful)
    {
        getAtoms();
        // Go put the atoms in the right zone.
        if (robot.isPlayingRightSide())
        {
            suctionOrder[0] = 0;
            suctionOrder[1] = 0;
            suctionOrder[2] = 1;
        }
        else
        {
            suctionOrder[0] = 2;
            suctionOrder[1] = 0;
            suctionOrder[2] = 1;
        }

        // Move to blue zone.
        targetPosition = robot.getCurrentPosition();
        targetPosition.y = 950;
        traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition, 0, true);
        endPosition = targetPosition;
        endPosition.x = 450 + CHASSIS_FRONT;
        traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
        robot.setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot.waitForTrajectoryFinished();
        robot.moveRail(0.3);
        robot.servos_.moveSuction(false);
        bool wasBlueDropped = false;
        if (wasMoveSuccessful)
        {
            // Drop blue atom.
            wasBlueDropped = true;

            robot.servos_.tapOpen();
            robot.servos_.openTube(suctionOrder[0]);
            usleep(300000);
            robot.servos_.tapClose();
            robot.servos_.closeTube(suctionOrder[0]);
            robot.updateScore(6);   // One correct drop.
            targetPosition = robot.getCurrentPosition();
            traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
            robot.setTrajectoryToFollow(traj);
            robot.waitForTrajectoryFinished();
        }

        // Go drop green and red atom.
        targetPosition = robot.getCurrentPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 750;
        targetPosition.y = 1400;
        positions.push_back(targetPosition);
        targetPosition.x = 450 + CHASSIS_FRONT;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.05);
        robot.setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot.waitForTrajectoryFinished();

        // Drop everything.
        robot.servos_.tapOpen();
        robot.servos_.openTube(0);
        robot.servos_.openTube(1);
        robot.servos_.openTube(2);
        robot.updateScore(6);   // One correct drop.
        usleep(500000);
        // Update score if needed.
        if (!wasBlueDropped)
            robot.updateScore(1);
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -80);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
    }


    //**********************************************************
    // Drop the blue atom in the particle accelerator.
    //**********************************************************
    targetPosition = robot.getCurrentPosition();
    positions.clear();
    positions.push_back(targetPosition);
    targetPosition.x = 1300;
    positions.push_back(targetPosition);
    targetPosition.x = 1600;
    targetPosition.y = 2000 - CHASSIS_WIDTH - 145;
    positions.push_back(targetPosition);
    targetPosition.x = 1625;
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
    wasMoveSuccessful = robot.wasTrajectoryFollowingSuccessful();
    robot.servos_.raiseArms(robot.isPlayingRightSide());
    if (wasMoveSuccessful)
    {
        robot.updateScore(20); // 20 pts: atom + goldium release.

        //**********************************************************
        // Get the goldium atom.
        //**********************************************************
        robot.moveRail(1.0);
        robot.servos_.moveSuction(true);
        targetPosition = robot.getCurrentPosition();
        targetPosition.x = 3000 - 770;
        targetPosition.y = 1650;
        traj = miam::trajectory::computeTrajectoryStraightLineToPoint(robot.getCurrentPosition(), targetPosition);
        endPosition = targetPosition;
        endPosition.y = 2000 - CHASSIS_FRONT - 100;
        traj = traj + miam::trajectory::computeTrajectoryStraightLineToPoint(targetPosition, endPosition);
        robot.setTrajectoryToFollow(traj);
        wasMoveSuccessful = robot.waitForTrajectoryFinished();
        robot.servos_.foldArms();

        if (wasMoveSuccessful)
        {
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
            robot.updateScore(20); // 20 pts for goldium drop.
            robot.servos_.openTube(2);
            robot.servos_.tapOpen();
        }
    }

    std::cout << "Strategy thread ended" << std::endl;
}
