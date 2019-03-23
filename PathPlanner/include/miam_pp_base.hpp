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


using namespace miam;
using namespace miam::trajectory;


namespace miam_pp {

    /**
     * A list of positions: ((x, y), (x, y)...).
     */
    typedef std::vector<RobotPosition > WayPointList;

    /**
     * A trajectory: ((t, state, control), (t, state, control)...).
     */
    typedef std::vector<::TrajectoryPoint > TrajectoryVector;


    SampledTrajectory get_resampled_trajectory_main_robot(
        WayPointList waypoint_list,
        TrajectoryPoint first_trajectory_point,
        TrajectoryPoint last_trajectory_point
    );

    SampledTrajectory get_planned_trajectory_main_robot(
        WayPointList waypoint_list,
        TrajectoryPoint first_trajectory_point,
        TrajectoryPoint last_trajectory_point,
        bool plot = false,
        bool verbose = false,
        bool vlin_free_at_end = false
    );


    /**
     * Given a current state, a list of waypoints and a planning horizon,
     * compute a suitable trajectory. Assume the dynamics are those of the
     * main robot.
     */
    SampledTrajectory get_planned_trajectory_main_robot_from_init(
        Trajectory& init_trajectory,
        TrajectoryPoint first_trajectory_point,
        TrajectoryPoint last_trajectory_point,
        bool plot = false,
        bool verbose = false,
        bool vlin_free_at_end = false,
        int N = -1
    );

}

#endif
