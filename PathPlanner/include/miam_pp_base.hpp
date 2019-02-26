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


using std::vector;

using miam::RobotPosition;
using miam::trajectory::TrajectoryPoint;
using miam::trajectory::SampledTrajectory;


namespace miam_pp {

/**
 * A list of positions: ((x, y), (x, y)...).
 */
typedef vector<RobotPosition > WayPointList;

//~ /**
 //~ * A trajectory: ((t, state, control), (t, state, control)...).
 //~ */
//~ typedef vector<TrajectoryPoint > std::vector<TrajectoryPoint >;



SampledTrajectory get_init_trajectory_from_waypoint_list(
    WayPointList waypoint_list,
    TrajectoryPoint first_trajectory_point,
    TrajectoryPoint last_trajectory_point
);


/**
 * Given a current state, a list of waypoints and a planning horizon,
 * compute a suitable trajectory. Assume the dynamics are those of the
 * main robot.
 */
SampledTrajectory get_planned_trajectory_main_robot(
    SampledTrajectory& init_trajectory,
    bool plot = false,
    bool verbose = false,
    bool vlin_free_at_end = false
);

}

#endif
