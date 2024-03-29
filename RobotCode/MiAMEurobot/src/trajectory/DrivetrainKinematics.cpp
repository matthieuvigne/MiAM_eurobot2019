/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/trajectory/DrivetrainKinematics.h"
#include <cmath>

DrivetrainKinematics::DrivetrainKinematics():
        motorWheelRadius_(1.0),
        motorWheelSpacing_(1.0),
        encoderWheelRadius_(1.0),
        encoderWheelSpacing_(1.0)
{
}

DrivetrainKinematics::DrivetrainKinematics(double const& motorWheelRadiusIn,
                                           double const& motorWheelSpacingIn,
                                           double const& encoderWheelRadiusIn,
                                           double const& encoderWheelSpacingIn):
        motorWheelRadius_(motorWheelRadiusIn),
        motorWheelSpacing_(motorWheelSpacingIn),
        encoderWheelRadius_(encoderWheelRadiusIn),
        encoderWheelSpacing_(encoderWheelSpacingIn)
{
}

BaseSpeed DrivetrainKinematics::forwardKinematics(WheelSpeed const& wheelSpeedIn, bool const& useEncoders)
{
    BaseSpeed speed;
    double wheelRadius = (useEncoders ? encoderWheelRadius_ : motorWheelRadius_);
    double wheelSpacing = (useEncoders ? encoderWheelSpacing_ : motorWheelSpacing_);

    // Linear velocity: average of both velocities.
    speed.linear = (wheelSpeedIn.right + wheelSpeedIn.left) / 2.0 * wheelRadius;
    // Angular velocity: difference of both velocities over wheel spacing.
    speed.angular = (wheelSpeedIn.right - wheelSpeedIn.left) / 2.0 * wheelRadius / wheelSpacing;
    return speed;
}

WheelSpeed DrivetrainKinematics::inverseKinematics(BaseSpeed const& baseSpeedIn)
{
    WheelSpeed speed;
    speed.right = (baseSpeedIn.linear + motorWheelSpacing_ * baseSpeedIn.angular) / motorWheelRadius_;
    speed.left = (baseSpeedIn.linear - motorWheelSpacing_ * baseSpeedIn.angular) / motorWheelRadius_;
    return speed;
}


void DrivetrainKinematics::integratePosition(WheelSpeed const& wheelSpeedIn, miam::RobotPosition & positionInOut, bool const& useEncoders)
{
    // Compute base velocity.
    BaseSpeed speed = forwardKinematics(wheelSpeedIn, useEncoders);
    // Integrate into position.
    positionInOut.theta += speed.angular;
    positionInOut.x += std::cos(positionInOut.theta) * speed.linear;
    positionInOut.y += std::sin(positionInOut.theta) * speed.linear;
}
