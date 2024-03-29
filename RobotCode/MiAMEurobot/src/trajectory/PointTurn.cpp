/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/trajectory/PointTurn.h"
#include "MiAMEurobot/trajectory/Utilities.h"

#include <cmath>


namespace miam{
    namespace trajectory{
        PointTurn::PointTurn(RobotPosition const& startPoint,
                             double const& endAngle,
                             double maxVelocity,
                             double maxAcceleration):
             endAngle_(endAngle),
             maxVelocity_(maxVelocity),
             maxAcceleration_(maxAcceleration)
        {
            make(startPoint);
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

        void PointTurn::make(RobotPosition const& startPoint)
        {
            startPoint_ = startPoint;
            motionSign_ = 1.0;
            // Create trapezoid.
            double length = moduloTwoPi(endAngle_ - startPoint.theta);
            if(length < 0)
                motionSign_ = -1.0;

            // Compute max angular velocity and acceleration, taking into account wheel spacing.
            double maxRobotAngularVelocity = maxVelocity_ / config::robotWheelSpacing;
            double maxRobotAngularAcceleration = maxAcceleration_ / config::robotWheelSpacing;

            trapezoid_ = Trapezoid(length, 0.0, 0.0, maxRobotAngularVelocity, maxRobotAngularAcceleration);
            duration_ = trapezoid_.getDuration();

        }


        void PointTurn::replanify(double const& replanificationTime)
        {
            RobotPosition startPoint = getCurrentPoint(replanificationTime).position;
            make(startPoint);
        }

    }
}
