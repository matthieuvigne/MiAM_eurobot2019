#include <vector>
#include <tuple>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/SampledTrajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>

using namespace miam::trajectory;

/**
 * Initializes the MPC problem with the start of a given trajectory.
 * Must be called before performing the first iteration, for the other
 * runs assume the solver is already initialized.
 */
void initialize_MPC_problem(
    Trajectory* reference_trajectory,
    double current_time
);


/**
 * Given a current point and a reference_trajectory trajectory, compute  
 * a suitable update for the trajectory. Assume the dynamics are those 
 * of the main robot.
 */
TrajectoryPoint solve_MPC_problem(
    Trajectory* reference_trajectory,
    TrajectoryPoint current_trajectory_point,
    double current_time
);
