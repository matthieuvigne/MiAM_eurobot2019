#include <string>
#include <miam_pp.hpp>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/io/IOTrajectory.h>

using namespace std;

int main( ){
    
    
    // Computing the solution of a simple waypoint problem
    
    string trajectory_name = "zigzag_trajectory";
    
    cout << "Computing trajectory " << trajectory_name << endl;
    
    
    //// First half
    
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
    last_trajectory_point.linearVelocity = 200.0;
    last_trajectory_point.angularVelocity = 0.0;
    
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
    
    //// Second half
    
    miam_pp::WayPointList waypoint_list_2 = miam_pp::WayPointList();
    waypoint_list_2.push_back(miam::RobotPosition(1000.0, 200.0, 1.0));
    waypoint_list_2.push_back(miam::RobotPosition(1200.0, 275.0, 0.5));
    waypoint_list_2.push_back(miam::RobotPosition(1500.0, 300.0, 0.5));
    waypoint_list_2.push_back(miam::RobotPosition(2000.0, 400.0, 0.0));
    
    miam::trajectory::TrajectoryPoint first_trajectory_point_2;
    first_trajectory_point_2.position = waypoint_list_2.front();
    first_trajectory_point_2.linearVelocity = 200.0;
    first_trajectory_point_2.angularVelocity = 0.0;
    miam::trajectory::TrajectoryPoint last_trajectory_point_2;
    last_trajectory_point_2.position = waypoint_list_2.back();
    last_trajectory_point_2.linearVelocity = 0.0;
    last_trajectory_point_2.angularVelocity = 0.0;
    
     /* 
     * Trajectory planning 
     */
    miam::trajectory::SampledTrajectory sampled_trajectory_2 = miam_pp::get_planned_trajectory_main_robot(
        waypoint_list_2,
        first_trajectory_point_2,
        last_trajectory_point_2,
        true, // Plot output
        true  // Print output
    );
    
    //// Combine trajectories
    miam_pp::TrajectoryVector output_trajectory_vector = sampled_trajectory.getUnderlyingPoints();
    // Remove last point (duplicate)
    output_trajectory_vector.pop_back();
    // Concatenate with second half
    miam_pp::TrajectoryVector output_trajectory_vector_2 = sampled_trajectory_2.getUnderlyingPoints();
    output_trajectory_vector.insert(
        output_trajectory_vector.end(),
        output_trajectory_vector_2.begin(),
        output_trajectory_vector_2.end()
    );
    
    miam::trajectory::SampledTrajectory sampled_trajectory_final(
        output_trajectory_vector,
        sampled_trajectory.getDuration() + sampled_trajectory_2.getDuration()
    );
    
    miam::io::writeSampledTrajectoryToFile(sampled_trajectory_final, trajectory_name);

	return 0;
}
