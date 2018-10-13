#include "MiAMEurobot/trajectory/PointTurn.h"
#include "MiAMEurobot/trajectory/Utilities.h"

#include <cmath>


namespace miam{
	namespace trajectory{
		PointTurn::PointTurn(RobotPosition const& startPoint,
							 double const& endAngle,
		                     double maxVelocity,
		                     double maxAcceleration):
		     startPoint_(startPoint),
		     motionSign_(1.0)
		{
			// Create trapezoid.
			double length = moduloTwoPi(endAngle - startPoint.theta);
			if(length < 0)
				motionSign_ = -1.0;

			// Compute max angular velocity and acceleration, taking into account wheel spacing.
			double maxRobotAngularVelocity = maxVelocity / config::robotWheelSpacing;
			double maxRobotAngularAcceleration = maxAcceleration / config::robotWheelSpacing;

			trapezoid_ = Trapezoid(length, 0.0, 0.0, maxRobotAngularVelocity, maxRobotAngularAcceleration);
			duration_ = trapezoid_.getDuration();
		}

		TrajectoryPoint PointTurn::getCurrentPoint(double const& currentTime)
		{
			TrajectoryPoint output;
			output.position = startPoint_;

			TrapezoidState state = trapezoid_.getState(currentTime);
			output.position.theta += motionSign_ * state.position;
			output.angularVelocity = motionSign_ * state.velocity;

			return output;
		}

	}
}
