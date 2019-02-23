#include <vector>
#include <tuple>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/SampledTrajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>

/**
 * Given a current point and a reference_trajectory trajectory, compute a suitable 
 * update for the trajectory. Assume the dynamics are those of the
 * main robot.
 */
miam::trajectory::TrajectoryPoint solve_MPC_problem(
    miam::trajectory::Trajectory* reference_trajectory,
    miam::trajectory::TrajectoryPoint current_trajectory_point,
    double current_time
);
