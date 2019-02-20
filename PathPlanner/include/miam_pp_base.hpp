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


// Define the robot dimensions if Robot.h was not defined
#ifndef ROBOT_H
namespace robotdimensions
{
    double const wheelRadius = 50.8; ///< Wheel radius, in mm.
    double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
    double const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
    double const encoderWheelSpacing = 133.0; ///< Encoder wheel spacing from robot center, in mm.

    double const stepSize = 2 * 3.14159265359 / 600.0; ///< Size of a motor step, in rad.

    double const maxWheelSpeed = 400; ///< Maximum wheel speed, in mm/s.
    double const maxWheelAcceleration = 400; ///< Maximum wheel acceleration, in mm/s^2.
}
#endif


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
    bool verbose = false
);

}

#endif
