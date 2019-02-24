#ifndef MIAM_PP_BASE
#define MIAM_PP_BASE

/**
 * Define generic path-planning problems
 * 
 */

#include <vector>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/SampledTrajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>





namespace miam_pp {

/**
 * A list of positions: ((x, y), (x, y)...).
 */
typedef std::vector<miam::RobotPosition > WayPointList;

/**
 * A trajectory: ((t, state, control), (t, state, control)...).
 */
typedef std::vector<miam::trajectory::TrajectoryPoint > TrajectoryVector;


/**
 * Given a current state, a list of waypoints and a planning horizon,
 * compute a suitable trajectory. Assume the dynamics are those of the
 * main robot.
 */
miam::trajectory::SampledTrajectory get_planned_trajectory_main_robot(
    WayPointList waypoint_list,
    miam::trajectory::TrajectoryPoint first_trajectory_point,
    miam::trajectory::TrajectoryPoint last_trajectory_point,
    bool plot = false,
    bool verbose = false,
    bool vlin_free_at_end = false
);

}

#endif
