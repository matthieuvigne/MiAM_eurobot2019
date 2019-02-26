#include <memory>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include <miam_pp.hpp>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>
#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>
#include <MiAMEurobot/io/IOTrajectory.h>


// Import robotdimensions
#include <MainRobotCode/include/Robot.h>

using namespace robotdimensions;


DrivetrainKinematics drivetrain_kinematics_2(
    wheelRadius,
    wheelSpacing,
    encoderWheelRadius,
    encoderWheelSpacing
);

using namespace std;

int main( ){
    
    // Some calculus tests
    
    miam::RobotPosition test = miam::RobotPosition(1.0, 2.0, 0.0);
    miam::RobotPosition test2 = miam::RobotPosition(3.0, 2.0, 0.0);
    
    
    BaseSpeed base_speed(1234.0, 0.2345);
    cout << base_speed.linear << " " << base_speed.angular << endl;
    
    WheelSpeed ws1 = drivetrain_kinematics_2.inverseKinematics(base_speed);
    std::cout << ws1.right << " " << ws1.left << endl;
    
    BaseSpeed bs1 = drivetrain_kinematics_2.forwardKinematics(
        drivetrain_kinematics_2.inverseKinematics(base_speed)
    );
    cout << bs1.linear << " " << bs1.angular << endl;
    
    
    WheelSpeed wheel_speed(1234.0, 2355.0);
    cout << wheel_speed.right << " " << wheel_speed.left << endl;
    
    BaseSpeed bs2 = drivetrain_kinematics_2.forwardKinematics(wheel_speed);
    std::cout << bs2.linear << " " << bs2.angular << endl;
    
    WheelSpeed ws2 = drivetrain_kinematics_2.inverseKinematics(
        drivetrain_kinematics_2.forwardKinematics(wheel_speed)
    );
    cout << ws2.right << " " << ws2.left << endl;
    
    
    
    std::cout << "RobotPosition: " << test << std::endl;
    std::cout << "RobotPosition: " << test-test2 << std::endl;
    std::cout << "RobotPosition: " << test2-test << std::endl;
    std::cout << "RobotPosition: " << 2.0*test << std::endl;
    std::cout << "RobotPosition: " << test*2.0 << std::endl;
    std::cout << "RobotPosition: " << test/3.0 << std::endl;
    
    miam::RobotPosition test3 = 2.0*test;
    
    std::cout << "RobotPosition: " << test3 << std::endl;
    std::cout << "RobotPosition: " << test3*2.0/3.0 << std::endl;
    
    std::cout << "Distance: " << miam::trajectory::distance(test, test3) << std::endl;
    
    miam_pp::Hello_World();
    
    // Computing the solution of a simple waypoint problem
    
    miam_pp::WayPointList waypoint_list = miam_pp::WayPointList();
    waypoint_list.push_back(miam::RobotPosition(0.0, 0.0, 0.0));
    waypoint_list.push_back(miam::RobotPosition(500.0, 100.0, 0.5));
    waypoint_list.push_back(miam::RobotPosition(800.0, 125.0, 0.5));
    waypoint_list.push_back(miam::RobotPosition(1000.0, 200.0, 1.0));

    miam::trajectory::TrajectoryPoint first_trajectory_point;
    first_trajectory_point.position = waypoint_list.front();
    first_trajectory_point.linearVelocity = 0.0;
    first_trajectory_point.angularVelocity = 0.0;
    miam::trajectory::TrajectoryPoint last_trajectory_point;
    last_trajectory_point.position = waypoint_list.back();
    last_trajectory_point.linearVelocity = 0.0;
    last_trajectory_point.angularVelocity = 0.0;
    
    /* 
     * Trajectory planning 
     */
     
    miam::trajectory::SampledTrajectory init_trajectory = miam_pp::get_init_trajectory_from_waypoint_list(
        waypoint_list,
        first_trajectory_point,
        last_trajectory_point
    );
    
    int M = init_trajectory.getUnderlyingPoints().size();
    
    std::cout << "M= " << M << std::endl;
    
    int N = std::floor(init_trajectory.getDuration() / 0.01);
    double tstep = init_trajectory.getDuration() / N;
    
    std::cout << "N= " << N << std::endl;
    std::cout << "tstep= " << tstep << std::endl;
    for (int i=0; i < N+1; i++)
    {
        miam::trajectory::TrajectoryPoint _tp = init_trajectory.getCurrentPoint(i * tstep);
        std::cout << _tp.position << ", v=" << _tp.linearVelocity << ", w=" << _tp.angularVelocity << std::endl;
    }
    
    miam::trajectory::SampledTrajectory sampled_trajectory = miam_pp::get_planned_trajectory_main_robot(
        init_trajectory,
        true, // Plot output
        true  // Print output
	);
    
    //~ int N = std::floor(sampled_trajectory.getDuration() / 0.01);
    //~ double tstep = sampled_trajectory.getDuration() / N;
    
    //~ std::cout << "N= " << N << std::endl;
    //~ std::cout << "tstep= " << tstep << std::endl;
    
    //~ for (int i=0; i < N+1; i++)
    //~ {
        //~ miam::trajectory::TrajectoryPoint _tp = sampled_trajectory.getCurrentPoint(i * tstep);
        //~ std::cout << _tp.position << ", v=" << _tp.linearVelocity << ", w=" << _tp.angularVelocity << std::endl;
    //~ }
    
    miam::io::writeSampledTrajectoryToFile(sampled_trajectory, "test_sample_trajectory");

	return 0;
}
