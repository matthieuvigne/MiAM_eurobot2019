#include <string>
#include <miam_pp.hpp>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/io/IOTrajectory.h>

using namespace std;

int main( ){
    
    
    // Computing the solution of a simple waypoint problem
    
    string trajectory_name = "curved_trajectory";
    
    cout << "Computing trajectory " << trajectory_name << endl;
    
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
    miam::trajectory::SampledTrajectory sampled_trajectory = miam_pp::get_planned_trajectory_main_robot(
        init_trajectory,
        true, // Plot output
        true  // Print output
	);
    
    miam::io::writeSampledTrajectoryToFile(sampled_trajectory, trajectory_name);

	return 0;
}
