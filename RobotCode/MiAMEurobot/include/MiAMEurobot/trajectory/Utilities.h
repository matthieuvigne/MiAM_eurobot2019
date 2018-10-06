/// \file trajectory/Utilities.h
/// \brief Utility functions for trajectory generation.
#ifndef MIAM_TRAJECTORY_UTILITIES
#define MIAM_TRAJECTORY_UTILITIES

	#include "MiAMEurobot/trajectory/Trajectory.h"

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
		}
	}
#endif
