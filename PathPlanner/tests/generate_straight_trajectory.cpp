#include <string>
#include <miam_pp.hpp>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/io/IOTrajectory.h>

using namespace std;

int main( ){
    
    
    // Computing the solution of a simple waypoint problem
    
    string trajectory_name = "straight_trajectory";
    
    cout << "Computing trajectory " << trajectory_name << endl;
    
    miam_pp::WayPointList waypoint_list = miam_pp::WayPointList();
    waypoint_list.push_back(miam::RobotPosition(0.0, 0.0, 0.00));
    waypoint_list.push_back(miam::RobotPosition(1000.0, 0.00, 0.00));

    miam::trajectory::TrajectoryPoint first_trajectory_point;
    first_trajectory_point.position = waypoint_list.front();
    first_trajectory_point.linearVelocity = 0.00;
    first_trajectory_point.angularVelocity = 0.00;
    miam::trajectory::TrajectoryPoint last_trajectory_point;
    last_trajectory_point.position = waypoint_list.back();
    last_trajectory_point.linearVelocity = 0.00;
    last_trajectory_point.angularVelocity = 0.00;
    
    /* 
     * Trajectory planning 
     */
    miam::trajectory::SampledTrajectory sampled_trajectory = miam_pp::get_planned_trajectory_main_robot(
        waypoint_list,
        first_trajectory_point,
        last_trajectory_point,
        true, // Plot output
        true  // Print output
);
    
    miam::io::writeSampledTrajectoryToFile(sampled_trajectory, trajectory_name);

	return 0;
}
