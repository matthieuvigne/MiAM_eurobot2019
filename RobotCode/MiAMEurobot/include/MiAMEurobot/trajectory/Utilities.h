/// \file trajectory/Utilities.h
/// \brief Utility functions for trajectory generation.
#ifndef MIAM_TRAJECTORY_UTILITIES
#define MIAM_TRAJECTORY_UTILITIES

	#include "MiAMEurobot/trajectory/Trajectory.h"
	#include <vector>
	#include <memory>

	namespace miam{
		namespace trajectory{

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
			/// \param[in] backward If translation should be done backward.
			/// \return Vector of pointer toward two trajectories: rotation then translation.
			std::vector<std::shared_ptr<Trajectory>> computeTrajectoryStaightLineToPoint(RobotPosition const& startPosition,
			                                                              RobotPosition const& endPosition,
			                                                              bool const& backward = false);
		}
	}
#endif
