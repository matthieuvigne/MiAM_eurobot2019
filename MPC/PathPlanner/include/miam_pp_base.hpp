/**
 * Define generic path-planning problems
 * 
 */
 
#include <vector>
#include <tuple>
 
#ifndef real
	#define real double
#endif

namespace miam_pp {

typedef std::vector<real > Vector;


/**
 * A spatial position (x, y...)
 */
typedef Vector Position;

/**
 * A state vector of a given model.
 */
typedef Vector State;

/**
 * A control vector of a given model.
 */
typedef Vector Control;

/**
 * A list of positions: ((x, y), (x, y)...).
 */
typedef std::vector<Position > WayPointList;

/**
 * An instantaneous representation (t, state, control).
 */
typedef std::tuple<float, State, Control > TrajectoryPoint;

/**
 * A trajectory: ((t, state, control), (t, state, control)...).
 */
typedef std::vector<TrajectoryPoint > Trajectory;


/**
 * Given a current state, a list of waypoints and a planning horizon,
 * compute a suitable trajectory. Assume the dynamics are those of the
 * main robot.
 */
Trajectory get_planned_trajectory_main_robot(
    State current_state,
    WayPointList waypoint_list,
    float planning_horizon
);

}
