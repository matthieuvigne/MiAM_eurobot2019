/// \file trajectory/Trajectory.h
/// \brief Definition of a trajectory, and associated elements.
///
/// \details What we call a trajectory is an object along wich we can servo the robot. It thus groups together the
///          notion of path (i.e. curve on the table, including robot angle), and of time parametrization (in order
///          to allow trajectory tracking in itself).
#ifndef MIAM_TRAJECTORY_TRAJECTORY
#define MIAM_TRAJECTORY_TRAJECTORY

	#include <glib.h>

	namespace miam{
		namespace trajectory{

			// Config variables, used for trajectory generation.
			namespace config
			{
				extern double maxLinearVelocity;	 ///< Maximum linear velocity along a trajectory, mm/s.
				extern double maxLinearAcceleration; ///< Maximum linear acceleration of a trajectory, mm/s2.
				extern double maxAngularVelocity;	 ///< Maximum angular velocity along a trajectory, rad/s.
				extern double maxAngularAcceleration; ///< Maximum angular acceleration of a trajectory, rad/s2.
			}

			/// \brief Robot coordinates on the table.
			struct RobotPosition
			{
				double x;	///< X coordinate of the robot, in mm.
				double y;	///< Y coordinate of the robot, in mm. Notice that y axis is taken positive when pointing downward.
				double theta;	///< Angle of the robot, in rad.

				/// \brief Default constructor.
				RobotPosition():
					x(0.0),
					y(0.0),
					theta(0.0)
				{}
			};


			/// \brief A trajectory point, containing everything for servoing along this trajectory.
			struct TrajectoryPoint{
				RobotPosition position; ///< Trajectory point in the table.
				double linearVelocity; ///< Linear velocity along the trajectory, at the current point.
				double angularVelocity; ///< Angular velocity along the trajectory, at the current point.
				bool isTrajectoryDone; ///< If the trajectory is done playing at the current point.

				/// \brief Default constructor.
				TrajectoryPoint():
			    position(),
			    linearVelocity(0.0),
			    angularVelocity(0.0),
			    isTrajectoryDone(false)
				{}
			};


			class Trajectory
			{
				public:
					Trajectory();

					/// \brief Get trajectory point at current time.
					///
					/// \param[in] currentTime Time relative to trajectory start, in seconds.
					/// \return The current trajectory point.
					virtual TrajectoryPoint getCurrentPoint(double const& currentTime) = 0;

					/// \brief Get trajectory duration, in seconds.
					/// \return Trajectory duration.
					double getDuration();

				protected:
					double duration_; ///< Trajectory duration.
			};
		}
	}
#endif
