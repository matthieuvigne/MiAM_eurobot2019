#include "MiAMEurobot/trajectory/StraightLine.h"
#include "MiAMEurobot/trajectory/Utilities.h"

#include <cmath>


namespace miam{
	namespace trajectory{
		StraightLine::StraightLine(RobotPosition const& startPoint,
		                           RobotPosition const& endPoint,
		                           double const& startVelocity,
		                           double const& endVelocity,
		                           bool const& backward,
		                           double maxVelocity,
		                           double maxAcceleration):
		     startPoint_(startPoint),
		     motionSign_(1.0)
		{
			if(backward)
				motionSign_ = -1.0;
			// Create trapezoid.
			double length = distance(startPoint, endPoint);
			trapezoid_ = Trapezoid(length, startVelocity, endVelocity, maxVelocity, maxAcceleration);

			duration_ = trapezoid_.getDuration();

			// Compute angle.
			startPoint_.theta = computeShortestAngle(startPoint, endPoint);

			if(backward)
			{
				// Add or remove pi if going backward.
				if(startPoint_.theta < 0)
					startPoint_.theta += G_PI;
				else
					startPoint_.theta -= G_PI;
			}
		}

		TrajectoryPoint StraightLine::getCurrentPoint(double const& currentTime)
		{
			TrajectoryPoint output;
			output.position = startPoint_;
			TrapezoidState state = trapezoid_.getState(currentTime);

			output.isTrajectoryDone = state.done;
			output.linearVelocity = motionSign_ * state.velocity;

			// Compute position, remember y is negative.
			output.position.x += motionSign_ * state.position * std::cos(startPoint_.theta);
			output.position.y -= motionSign_ * state.position * std::sin(startPoint_.theta);

			return output;
		}

	}
}
