#ifndef MIAM_MPC
#define MIAM_MPC

#include <vector>
#include <tuple>

#include <MiAMEurobot/trajectory/RobotPosition.h>
#include <MiAMEurobot/trajectory/Trajectory.h>
#include <MiAMEurobot/trajectory/SampledTrajectory.h>
#include <MiAMEurobot/trajectory/Utilities.h>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>


using namespace miam::trajectory;

namespace miam {
    
    /**
     * Pure static class.
     * Handles the MPC solver initialization features and implements the
     * MPC solving scheme.
     */
    class MPCsolver {
        
        private:
            MPCsolver() {}
        
            /**
             * Initialize the ACADO workspace. Must be called before any other
             * function of the MPC solver.
             */
            static void initialize_MPC_solver();

            /**
             * Initializes the MPC problem with the start of a given trajectory.
             * Must be called before performing the first iteration, for the other
             * runs assume the solver is already initialized.
             */
            static void initialize_MPC_problem(
                Trajectory* reference_trajectory,
                double current_time
            );
            
            /**
             * MPC current trajectory 
             * Used to determine whether the solver should initialize a new traj
             * tracking problem.
             */
            static Trajectory* _mpc_current_trajectory;
            
            /**
             * MPC current state 
             * Used to determine whether the solver has been initialized once.
             */
            static bool _mpc_solver_initialized;

        public:
            /**
             * Given a current point and a reference_trajectory trajectory, compute  
             * a suitable update for the trajectory. Assume the dynamics are those 
             * of the main robot.
             */
            static TrajectoryPoint solve_MPC_problem(
                Trajectory* reference_trajectory,
                TrajectoryPoint current_trajectory_point,
                double current_time
            );

    };

}

#endif
