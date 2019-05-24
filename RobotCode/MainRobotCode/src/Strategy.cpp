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
        robot.moveRail(0.40);

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
    //~ RobotPosition endPosition = robot.getCurrentPosition();
    //~ std::cout << "Playing first trajectory" << std::endl;
    //~ endPosition.x += 500;
    //~ std::vector<std::shared_ptr<Trajectory>> traj;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ std::cout << "Playing second trajectory" << std::endl;
    //~ RobotPosition startPosition = endPosition;
    //~ endPosition.y += 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ std::cout << "Playing third trajectory" << std::endl;
    //~ endPosition.theta = M_PI_2;
    //~ startPosition = endPosition;
    //~ endPosition.x -= 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();
    //~ startPosition = endPosition;

    //~ std::cout << "Playing fourth trajectory" << std::endl;
    //~ endPosition.theta = M_PI;
    //~ startPosition = endPosition;
    //~ endPosition.y -= 500;
    //~ traj = miam::trajectory::computeTrajectoryStaightLineToPoint(startPosition, endPosition);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ usleep(200000);
    //~ printf("done\n");
    //~ while(true) ;;

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

    //~ robot.moveRail(1.0);
    //~ traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -30);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ robot.servos_.moveMiddleSuctionForDrop();
    //~ robot.servos_.turnOnPump();
    //~ robot.servos_.tapOpen();
    //~ robot.servos_.closeTube(0);
    //~ robot.servos_.openTube(1);
    //~ robot.servos_.closeTube(2);
    //~ usleep(2000000);
    //~ robot.servos_.closeTube(1);
    //~ robot.servos_.tapClose();
    //~ usleep(1000000);
    //~ robot.servos_.turnOffPump();
    //~ usleep(1000000);

    //~ traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 30);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();
    //~ robot.servos_.moveMiddleSuctionForDrop(true);
    //~ usleep(500000);

    //~ miam::trajectory::setTrajectoryGenerationConfig(0.3 * robotdimensions::maxWheelSpeedTrajectory,
                                                    //~ 0.3 * robotdimensions::maxWheelAccelerationTrajectory,
                                                    //~ robotdimensions::wheelSpacing);
    //~ traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -15);
    //~ miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    //~ robotdimensions::maxWheelAccelerationTrajectory,
                                                    //~ robotdimensions::wheelSpacing);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();
    //~ robot.moveRail(0.70);
    //~ traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 10);
    //~ robot.setTrajectoryToFollow(traj);
    //~ robot.waitForTrajectoryFinished();

    //~ robot.servos_.openTube(1);
    //~ robot.servos_.tapOpen();


    //~ while(true) ;;
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
    targetPosition.y = 2000 - CHASSIS_WIDTH - 120;
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

    //**********************************************************
    // Get the goldium atom.
    //**********************************************************
    targetPosition = robot.getCurrentPosition();

    while (true) ;;

    bool phaseSucceeded = true;

    if (phaseSucceeded)
    {
        // Handle goldium.
        // Drop current atom.
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 10);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();

        miam::trajectory::setTrajectoryGenerationConfig(0.3 * robotdimensions::maxWheelSpeedTrajectory,
                                                        0.3 * robotdimensions::maxWheelAccelerationTrajectory,
                                                        robotdimensions::wheelSpacing);
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -15);
        miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                        robotdimensions::maxWheelAccelerationTrajectory,
                                                        robotdimensions::wheelSpacing);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
        robot.moveRail(0.70);
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 10);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();

        robot.servos_.openTube(1);
        robot.servos_.tapOpen();


        robot.updateScore(20);
        targetPosition = robot.getCurrentPosition();
        // Go get goldium.
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
        targetPosition = traj.getEndPoint().position;
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 2500 - 275;
        positions.push_back(targetPosition);
        targetPosition.y = 2000 - CHASSIS_FRONT - 80;
        positions.push_back(targetPosition);
        traj = traj + miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.1);
        //~ phaseSucceeded = robot.followTrajectory(traj);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
    }
    if (phaseSucceeded)
    {
        // Grab goldium
        getAtoms(10, false);
        robot.updateScore(20);

        // Go put it down in the scale.
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
        targetPosition = robot.getCurrentPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 1300;
        targetPosition.y = 1200;
        positions.push_back(targetPosition);
        targetPosition.y = 450 + CHASSIS_FRONT;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.1);
        //~ phaseSucceeded = robot.followTrajectory(traj);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();
    }


    std::cout << "Strategy thread ended" << std::endl;
}
