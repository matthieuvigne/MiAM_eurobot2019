#include "MiAMEurobot/trajectory/Trajectory.h"

namespace miam{
	namespace trajectory{

		// Default trajectory config.
		namespace config
		{
			double maxLinearVelocity = 0.8 * 600.0;
			double maxLinearAcceleration = 0.8 * 700.0;
			// Max angular velocity, computed taking into account wheel diamter and spacing.
			double maxAngularVelocity = maxLinearVelocity / (10.0 * 10.0);
			double maxAngularAcceleration = maxLinearAcceleration / (10.0 * 10.0);
		}
		Trajectory::Trajectory()
		{
			duration_ = 0.0;
		}

		double Trajectory::getDuration()
		{
			return duration_;
		}
	}
}
