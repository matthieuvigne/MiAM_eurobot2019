#include "MiAMEurobot/trajectory/Utilities.h"
#include "MiAMEurobot/trajectory/StraightLine.h"
#include "MiAMEurobot/trajectory/PointTurn.h"
#include "MiAMEurobot/trajectory/ArcCircle.h"

#include <iostream>
#include <cmath>
#include <glib.h>

namespace miam{
	namespace trajectory{

		RobotPosition computeCircleCenter(RobotPosition const& startingPosition, double const& radius, rotationside const& side)
		{
			RobotPosition circleCenter;
			double centerAngle = startingPosition.theta + G_PI / 2.0;
			if(side == rotationside::RIGHT)
				centerAngle -= G_PI;
			circleCenter.x = startingPosition.x + std::abs(radius) * std::cos(centerAngle);
			circleCenter.y = startingPosition.y - std::abs(radius) * std::sin(centerAngle);
			circleCenter.theta = centerAngle - G_PI;
			return circleCenter;
		}


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
			return (first - second).norm();
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
		                                                             double const& endVelocity,
		                                                             bool const& backward)
		{
			std::vector<std::shared_ptr<Trajectory>> vector;
			std::shared_ptr<StraightLine> line(new StraightLine(startPosition, endPosition, 0.0, endVelocity, backward));

			// Get angle from straight line as rotation target.
			vector.push_back(std::shared_ptr<Trajectory>(new PointTurn(startPosition, line->getAngle())));
			vector.push_back(std::shared_ptr<Trajectory>(line));
			return vector;
		}

		std::vector<std::shared_ptr<Trajectory>> computeTrajectoryRoundedCorner(std::vector<RobotPosition> const& positions,
			                                                                    double radius,
			                                                                    double transitionVelocityFactor)
		{
			std::vector<std::shared_ptr<Trajectory>> trajectories;
			if(positions.size() < 2)
				return trajectories;
			// Compute the transition angular velocity.
			double factor = transitionVelocityFactor;
			if(factor < 0.0)
				factor = 0.0;
			else if(factor > 1.0)
				factor = 1.0;

			double transitionLinearVelocity = factor * config::maxWheelVelocity;
			double transitionAngularVelocity = transitionLinearVelocity / (std::abs(radius) + config::robotWheelSpacing);

			// Compute first rotation to be aligned with second point.
			std::vector<std::shared_ptr<Trajectory>> straightLine = computeTrajectoryStaightLineToPoint(positions.at(0),
				positions.at(1),
				transitionLinearVelocity);
			trajectories.push_back(straightLine[0]);

			// For each remaining pair of points, computed the line and rounded corner to go there.
			for(uint i = 1; i < positions.size() - 1; i++)
			{
				RobotPosition startPoint = trajectories.back()->getEndPoint().position;
				RobotPosition roundedCornerPoint = positions.at(i);
				RobotPosition endPoint = positions.at(i + 1);

				// Find position of the center of the circle.
				// Get vectors going from the corner to both points.
				RobotPosition firstVector = startPoint - roundedCornerPoint;
				double firstNorm = firstVector.norm();
				firstVector.normalize();
				RobotPosition secondVector = endPoint - roundedCornerPoint;
				double secondNorm = secondVector.norm();
				secondVector.normalize();

				// Find angle between both vectors.
				double angle = std::acos(firstVector.dot(secondVector));

				// Get distance from roundedCornerPoint to center along each vector, reducing the radius if it is too large.
				double coefficient = 1 / std::tan(angle / 2.0);

				double circleRadius = std::min(radius, std::min(firstNorm, secondNorm) / coefficient * 0.99);

				// Compute circle center.
				RobotPosition center = roundedCornerPoint + circleRadius * coefficient * (firstVector + secondVector);

				// Finally, compute point where the circle intersects both trajectories: get the first point and the
				// angle.
				RobotPosition circleIntersection = roundedCornerPoint + circleRadius * coefficient * firstVector;
				// Put back right angle.
				circleIntersection.theta = startPoint.theta;

				// Compute direction of the circle, based on cross-product.
				rotationside side = rotationside::RIGHT;
				if(firstVector.cross(secondVector) > 0.0)
					side = rotationside::LEFT;

				// Compute end angle.
				double endAngle = std::atan2(-secondVector.y, secondVector.x);
				if(side == rotationside::LEFT)
				{
					endAngle -= G_PI_2;
					std::cout << "right side" << std::endl;
				}
				else
				{
					endAngle += G_PI_2;
					std::cout << "right side" << std::endl;
				}
				// Compute trajectory.
				std::shared_ptr<StraightLine> line(new StraightLine(startPoint, circleIntersection, transitionLinearVelocity, transitionLinearVelocity));
				trajectories.push_back(line);
				std::shared_ptr<ArcCircle> circle(new ArcCircle(circleIntersection, circleRadius, side, endAngle, transitionAngularVelocity, transitionAngularVelocity));
				trajectories.push_back(circle);
			}
			// Append final straight line.
			RobotPosition currentPoint = trajectories.back()->getEndPoint().position;
			std::cout << "end line" <<  currentPoint.theta << std::endl;
			RobotPosition endPoint = positions.back();
			std::shared_ptr<StraightLine> line(new StraightLine(currentPoint, endPoint, transitionLinearVelocity, 0.0));
			trajectories.push_back(line);

			return trajectories;
		}

	}
}
