/// \file io/IOTrajectory.h
/// \brief IO functions for trajectories.
#ifndef MIAM_IO_IOTrajectory
#define MIAM_IO_IOTrajectory

    #include "MiAMEurobot/trajectory/SampledTrajectory.h"
    #include <string>
    #include <iostream>

    namespace miam{
        namespace io{
            
            trajectory::SampledTrajectory readSampledTrajectoryFromFile(std::string file_name);
            void writeSampledTrajectoryToFile(trajectory::SampledTrajectory trajectory, std::string file_name);
    
        }
    }

#endif
