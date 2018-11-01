/// \file trajectory/Utilities.h
/// \brief Utility functions for trajectory generation.
#ifndef MIAM_TRAJECTORY_UTILITIES
#define MIAM_TRAJECTORY_UTILITIES

	#include "MiAMEurobot/trajectory/Trajectory.h"
	#include "MiAMEurobot/trajectory/ArcCircle.h"
	#include <vector>
	#include <memory>

	namespace miam{
		namespace trajectory{

			/// \brief Compute coordinates of the center of a circle of a given radius tangent to the given position.
			///
			/// \param[in] startingPosition Starting position.
			/// \param[in] radius Circle radius.
			/// \param[in] side Rotation side.
			/// \return Center position, with angle set to match startingPosition angle when computing the coordinates
			///         of a point along the circle.
			RobotPosition computeCircleCenter(RobotPosition const& startingPosition,
			                                  double const& radius,
			                                  rotationside const& side);


			/// \brief Compute shortest angle between two points.
			/// \details Given two (x,y) coordinates, compute the line angle between them. Return the angle,
			///          modulo two pi, closest to the startPoint angle.
			///
			/// \param[in] startPoint Starting point.
			/// \param[in] startPoint End point.
			/// \return An angle value.
			double computeShortestAngle(RobotPosition startPoint, RobotPosition endPoint);

			/// \brief Return the distance between two point.
			///
			/// \param[in] first First point.
			/// \param[in] second Second point.
			/// \return Euclidean distance between these two points.
			double distance(RobotPosition const& first, RobotPosition const& second);

			/// \brief Return equivalent angle, modulo two pi.
			/// \return angle in ]-pi, pi]
			double moduloTwoPi(double angle);

			/// \brief Compute a rotation followed by a straight line to go from the start to end position.
			///
			/// \details This functions returns a length 2 vector of pointers: the first is the rotation required to
			///          go to the specified point in a straight line, while the second one is the line to that point.
			///
			/// \param[in] startPosition Starting position
			/// \param[in] endPosition Final position - angle is not taken into account.
			/// \param[in] endVelocity Final velocity.
			/// \param[in] backward If translation should be done backward.
			/// \return Vector of pointer toward two trajectories: rotation then translation.
			std::vector<std::shared_ptr<Trajectory>> computeTrajectoryStaightLineToPoint(RobotPosition const& startPosition,
			                                                                             RobotPosition const& endPosition,
			                                                                             double const& endVelocity = 0.0,
			                                                                             bool const& backward = false);

			/// \brief Given a list of points, compute a polyline going from the first point to the last, with rounded
			///        corners.
			///
			/// \details Given three points in the trajectory (a_{i-1}, a_i, a_{i+1}), the trajectory is a straight
			///          line to a_i, then a circle starting at a_i torward a_{i+1}, then a line to a_{i+}
			///
			/// \param[in] positions Vector of positions through which to go. Angles are not taken into account except
			///                      for the first position.
			/// \param[in] radius Circle radius - the same is used at each point.
			/// \param[in] transitionVelocityFactor Percentage of the maximum velocity along the circle at which to do the transition.
			/// \return Vector of pointer toward the full trajectory.
			std::vector<std::shared_ptr<Trajectory>> computeTrajectoryRoundedCorner(std::vector<RobotPosition> const& positions,
			                                                                        double radius,
			                                                                        double transitionVelocityFactor = 0.5);
		}
	}
#endif
