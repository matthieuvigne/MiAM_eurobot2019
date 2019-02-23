#include "MiAMEurobot/trajectory/SampledTrajectory.h"
#include <cmath>

#define MIAM_EUROBOT_SAMPLED_TRAJECTORY_RESAMPLE_TIMESTEP 0.01

using namespace std;

namespace miam{
    namespace trajectory{
        SampledTrajectory::SampledTrajectory(
            std::vector<TrajectoryPoint > sampledTrajectory,
            double duration
            ) : Trajectory()
        {
            duration_ = duration;
            sampledTrajectory_ = sampledTrajectory;
        }

        TrajectoryPoint SampledTrajectory::getCurrentPoint(double const& currentTime)
        {

            int N = sampledTrajectory_.size() - 1;

            if (currentTime >= duration_)
            {
                return sampledTrajectory_.back();
            }
            else if (currentTime <= 0.0)
            {
                return sampledTrajectory_.front();
            }

            int indexLow = std::floor(N * currentTime / duration_);
            int indexHigh = std::ceil(N * currentTime / duration_);

            TrajectoryPoint tpLow = sampledTrajectory_[indexLow];
            TrajectoryPoint tpHigh = sampledTrajectory_[indexHigh];

            double sampledTimestep = duration_ / N;

            double residue = (currentTime - indexLow * sampledTimestep) / sampledTimestep;

            double ponderationLow = 1.0 - residue;
            double ponderationHigh = residue;

            // Linear interpolation
            TrajectoryPoint output;
            output.position = ponderationLow * tpLow.position + ponderationHigh * tpHigh.position;
            output.linearVelocity = ponderationLow * tpLow.linearVelocity + ponderationHigh * tpHigh.linearVelocity;
            output.angularVelocity = ponderationLow * tpLow.angularVelocity + ponderationHigh * tpHigh.angularVelocity;

            return output;

        }
        
        void SampledTrajectory::replanify(double const& replanificationTime) 
        {
            // Create a new vector
            std::vector<TrajectoryPoint > newSampledTrajectory;
            
            // Do not do anything if the trajectory was finished
            if (getDuration() <= replanificationTime)
            {
                return;
            }
            
            // Number of resampling points
            int N = std::ceil((getDuration() - replanificationTime) / MIAM_EUROBOT_SAMPLED_TRAJECTORY_RESAMPLE_TIMESTEP);
            double newStep = (getDuration() - replanificationTime) / N;
            
            for (int i=0; i<N; i++) 
            {
                newSampledTrajectory.push_back(
                    getCurrentPoint(replanificationTime + i * newStep)
                );
            }
            
            // New time and new vector of trajectory points
            duration_ = getDuration() - replanificationTime;
            sampledTrajectory_ = newSampledTrajectory;
        }
    }
}
