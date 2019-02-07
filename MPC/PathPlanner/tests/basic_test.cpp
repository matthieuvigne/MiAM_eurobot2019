#include <memory>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include <miam_pp.hpp>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>

int main( ){
    
    // Some calculus tests
    
    miam::RobotPosition test = miam::RobotPosition(1.0, 2.0, 0.0);
    miam::RobotPosition test2 = miam::RobotPosition(3.0, 2.0, 0.0);
    
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
    miam::trajectory::SampledTrajectory sampled_trajectory = miam_pp::get_planned_trajectory_main_robot(
        waypoint_list,
        first_trajectory_point,
        last_trajectory_point,
        true, // Plot output
        true  // Print output
    );
    
    int N = std::floor(sampled_trajectory.getDuration() / 0.01);
    double tstep = sampled_trajectory.getDuration() / N;
    
    std::cout << "N= " << N << std::endl;
    std::cout << "tstep= " << tstep << std::endl;
    
    for (int i=0; i < N+1; i++)
    {
        miam::trajectory::TrajectoryPoint _tp = sampled_trajectory.getCurrentPoint(i * tstep);
        std::cout << _tp.position << ", v=" << _tp.linearVelocity << ", w=" << _tp.angularVelocity << std::endl;
    }
    
    /* 
     * Trajectory tracking 
     */
    
    // Perturbate the state at t=5.0
    double current_time = 1.3;
    miam::trajectory::TrajectoryPoint perturbated_current_state = sampled_trajectory.getCurrentPoint(current_time);
    
    std::cout << "Unperturbated state: " << std::endl;
    std::cout << perturbated_current_state.position << ", v=" << perturbated_current_state.linearVelocity << ", w=" << perturbated_current_state.angularVelocity << std::endl;
    
    perturbated_current_state.position.x += 15.21;
    perturbated_current_state.position.x -= 7.28;
    perturbated_current_state.position.theta += 0.05;
    perturbated_current_state.linearVelocity -= 12;
    perturbated_current_state.angularVelocity += 0.01;
    
    std::cout << "Perturbated state: " << std::endl;
    std::cout << perturbated_current_state.position << ", v=" << perturbated_current_state.linearVelocity << ", w=" << perturbated_current_state.angularVelocity << std::endl;
    
    miam::trajectory::TrajectoryPoint updated_current_state = miam_pp::solve_MPC_problem(
        sampled_trajectory,
        perturbated_current_state,
        current_time
        );
    
    std::cout << "Suggested next state some (1) steps after: " << std::endl;
    std::cout << updated_current_state.position << ", v=" << updated_current_state.linearVelocity << ", w=" << updated_current_state.angularVelocity << std::endl;
    
	return 0;
}
