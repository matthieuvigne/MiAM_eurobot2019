#include "MiAMEurobot/trajectory/Utilities.h"
#include "MiAMEurobot/trajectory/StraightLine.h"
#include "MiAMEurobot/trajectory/PointTurn.h"

#include <cmath>
#include <glib.h>

namespace miam{
	namespace trajectory{

		double computeShortestAngle(RobotPosition startPoint, RobotPosition endPoint)
		{
			// Switch y axis signs.
			startPoint.y = - startPoint.y;
			endPoint.y = - endPoint.y;

			// Compute line angle.
			double angle = std::atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x);

			// Return value in ]-pi, pi] of the original angle.
			return moduloTwoPi(angle - startPoint.theta);
		}


		double distance(RobotPosition const& first, RobotPosition const& second)
		{
			return std::sqrt((first.x - second.x) * (first.x - second.x) + (first.y - second.y) * (first.y - second.y));
		}

		double moduloTwoPi(double angle)
		{
			while(angle <= - G_PI)
				angle += 2 * G_PI;
			while(angle > G_PI)
				angle -= 2 * G_PI;
			return angle;
		}

		std::vector<std::shared_ptr<Trajectory>> computeTrajectoryStaightLineToPoint(RobotPosition const& startPosition,
		                                                             RobotPosition const& endPosition,
		                                                             bool const& backward)
		{
			std::vector<std::shared_ptr<Trajectory>> vector;
			std::shared_ptr<StraightLine> line(new StraightLine(startPosition, endPosition, 0.0, 0.0, backward));

			// Get angle from straight line as rotation target.
			vector.push_back(std::shared_ptr<Trajectory>(new PointTurn(startPosition, line->getAngle())));
			vector.push_back(std::shared_ptr<Trajectory>(line));
			return vector;
		}

	}
}
