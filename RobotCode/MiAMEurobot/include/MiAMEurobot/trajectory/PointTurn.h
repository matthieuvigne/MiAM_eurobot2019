/// \file trajectory/PointTurn.h
/// \brief In-place rotation of the robot.
#ifndef MIAM_TRAJECTORY_POINT_TURN
#define MIAM_TRAJECTORY_POINT_TURN

	#include "MiAMEurobot/trajectory/Trajectory.h"
	#include "MiAMEurobot/trajectory/Trapezoid.h"

	namespace miam{
		namespace trajectory{
			class PointTurn: public Trajectory
			{
				public:
					/// \brief Constructor.
					///
					/// \param[in] startPoint Trajectory starting point.
					/// \param[in] endAngle Ending angle - it will be taken modulo 2 pi.
					/// \param[in] maxVelocity Max velocity. Only absolute value is taken into account.
					/// \param[in] maxAcceleration Max acceleration. Only absolute value is taken into account.
					PointTurn(RobotPosition const& startPoint,
							  double const& endAngle,
							  double maxVelocity=config::maxAngularVelocity,
							  double maxAcceleration=config::maxAngularAcceleration);

					TrajectoryPoint getCurrentPoint(double const& currentTime);

				private:
					RobotPosition startPoint_; ///< Point where the trajectory started.
					Trapezoid trapezoid_; ///< Velocity trapezoid.
			};
		}
	}
#endif
