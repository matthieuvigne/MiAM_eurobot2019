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
        robot.moveRail(0.3);

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
       robot.moveRail(0.7);

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -moveAmount);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    robot.servos_.turnOffPump();
}

void matchStrategy()
{
    std::cout << "Strategy thread started." << std::endl;

    //~ while(true) ;;
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
    robot.moveRail(0.2);

    // Update config.
    // Increase wheel spacing to slow down rotations.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    TrajectoryVector traj;
    std::vector<RobotPosition> positions;

    // Go get first atoms.
    //~ targetPosition.y = 150;
    targetPosition.y = CHASSIS_FRONT + 50;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    getAtoms();
    robot.moveRail(0.1);
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
    robot.moveRail(0.4);
    robot.servos_.moveSuction(true);


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
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.1);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Grab atoms.
    getAtoms();

    // Go back, and push the atoms to the end of the red zone.
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -120);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    targetPosition = robot.getCurrentPosition();
    targetPosition.y = 1325;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Turn to put the atoms in the red zone.
    std::shared_ptr<ArcCircle> circle(new ArcCircle(robot.getCurrentPosition(), 150.0, rotationside::LEFT, M_PI_2));
    traj.clear();
    traj.push_back(circle);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    targetPosition = robot.getCurrentPosition();
    targetPosition.x -= 10;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Drop the red atoms.
    robot.servos_.moveSuction(false, false);
    robot.servos_.tapOpen();
    robot.servos_.openTube(0);
    robot.servos_.openTube(2);
    usleep(1000000);
    robot.servos_.tapClose();
    robot.servos_.closeTube(0);
    robot.servos_.closeTube(2);
    robot.updateScore(4 * 6 + 1); // Drop 5 atoms in the red zone.
    robot.servos_.moveSuction(true);
    robot.servos_.moveMiddleSuctionForDrop();
    robot.moveRail(1.0);

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Next phase: go reach the goldium.
    // Perform a choice based on oponent robot position.
    //~ if (robot.obstacleY_ < 1500 || robot.obstacleX_ > 1500)
    if (true)
    {
        // Shortest way: go along the limit.
        targetPosition = robot.getCurrentPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x += 300;
        targetPosition.y = 2000 - CHASSIS_WIDTH - 125;
        positions.push_back(targetPosition);
        targetPosition.x = 2000;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.1);
        targetPosition = traj.getEndPoint().position;
        RobotPosition endPosition = targetPosition;
        endPosition.y = 2000 - CHASSIS_FRONT - 44;
        traj = traj +  miam::trajectory::computeTrajectoryStaightLineToPoint(targetPosition, endPosition);
    }
    else
    {
        // Go below by the middle of the field.
        targetPosition = robot.getCurrentPosition();
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x += 80;
        targetPosition.y = 1400;
        positions.push_back(targetPosition);
        targetPosition.x = 2000;
        positions.push_back(targetPosition);
        targetPosition.y = 2000 - CHASSIS_FRONT - 44;
        positions.push_back(targetPosition);
        traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 80.0, 0.1);
    }
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    bool phaseSucceeded = true;

    if (phaseSucceeded)
    {
        // Handle goldium.
        // Drop current atom.
        targetPosition = robot.getCurrentPosition();
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -20);
        robot.setTrajectoryToFollow(traj);
        robot.waitForTrajectoryFinished();

        robot.servos_.tapOpen();
        robot.servos_.openTube(1);
        usleep(300000);
        robot.updateScore(20);
        targetPosition = robot.getCurrentPosition();
        // Go get goldium.
        traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
        targetPosition = traj.getEndPoint().position;
        positions.clear();
        positions.push_back(targetPosition);
        targetPosition.x = 2500 - 290;
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
