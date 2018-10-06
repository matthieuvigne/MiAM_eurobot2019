#include "MiAMEurobot/trajectory/PointTurn.h"
#include "MiAMEurobot/trajectory/Utilities.h"

#include <cmath>


namespace miam{
	namespace trajectory{
		PointTurn::PointTurn(RobotPosition const& startPoint,
							 double const& endAngle,
		                     double maxVelocity,
		                     double maxAcceleration):
		     startPoint_(startPoint)
		{
			// Create trapezoid.
			double length = moduloTwoPi(startPoint.theta - endAngle);
			trapezoid_ = Trapezoid(length, 0.0, 0.0, maxVelocity, maxAcceleration);
			duration_ = trapezoid_.getDuration();
		}

		TrajectoryPoint PointTurn::getCurrentPoint(double const& currentTime)
		{
			TrajectoryPoint output;
			output.position = startPoint_;

			TrapezoidState state = trapezoid_.getState(currentTime);
			output.position.theta += state.position;
			output.angularVelocity = state.velocity;
			output.isTrajectoryDone = state.done;

			return output;
		}

	}
}
