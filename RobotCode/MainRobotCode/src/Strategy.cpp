#include <unistd.h>

#include "Robot.h"

// Robot dimension.
double const CHASSIS_FRONT = 150.0;
double const CHASSIS_BACK = 150.0;
double const CHASSIS_WIDTH = 150.0;

using namespace miam::trajectory;

// Aquire the atoms, assuming the robot is 5cm in front of them.
void getAtoms()
{
    // Open tap, turn on pump, open all suction caps.
    robot.moveRail(0.4);

    robot.servos_.turnOnPump();
    robot.servos_.tapOpen();
    robot.servos_.moveSuction(true);
    for(int i = 0; i < 3; i++)
        robot.servos_.openTube(i);
    usleep(200000);

    // Move
    RobotPosition targetPosition = robot.getCurrentPosition();
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 50);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Close tap.
    robot.servos_.tapClose();
    usleep(500000);
    for(int i = 0; i < 3; i++)
        robot.servos_.closeTube(i);
    usleep(1000000);
    robot.moveRail(0.7);

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -50);
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
    targetPosition.x = CHASSIS_WIDTH + 25;
    targetPosition.y = 1100 + CHASSIS_FRONT + 25;
    targetPosition.theta = -M_PI_2;

    robot.resetPosition(targetPosition, true, true, true);

    // Update config.
    // Increase wheel spacing to slow down rotations.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    TrajectoryVector traj;
    std::vector<RobotPosition> positions;

    robot.updateScore(40); // Experiment points.

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
    targetPosition.y = 1700;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0, true);
    robot.setTrajectoryToFollow(traj);
    // Atom drop
    while (!robot.isTrajectoryFinished())
    {
        targetPosition = robot.getCurrentPosition();
        if (targetPosition.y > 950)
        {
            robot.servos_.tapOpen();
            robot.servos_.openTube(0);
        }
        if (targetPosition.y > 1300)
            robot.servos_.openTube(1);
        if (targetPosition.y > 1550)
            robot.servos_.openTube(2);
        usleep(20000);
    }
    robot.updateScore(18);

    // Go get left set of three atoms, following a curved trajectory.
    targetPosition = robot.getCurrentPosition();
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
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 100.0, 0.1);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Go back, and push the atoms to the end of the red zone.
    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -150);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();
    targetPosition = robot.getCurrentPosition();
    targetPosition.y = 1400;
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
    targetPosition.x -= 50;
    traj = miam::trajectory::computeTrajectoryStaightLineToPoint(robot.getCurrentPosition(), targetPosition, 0.0);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    // Drop the red atoms.
    robot.updateScore(4 * 6 + 1); // Drop 5 atoms in the red zone.

    targetPosition = robot.getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, -100);
    robot.setTrajectoryToFollow(traj);
    robot.waitForTrajectoryFinished();

    std::cout << "Strategy thread ended" << std::endl;
}
