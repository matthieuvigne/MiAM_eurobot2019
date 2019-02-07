#include <miam_pp_base.hpp>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */


/*
 * Parameters with which the custom solver was compiled
 */
#define DELTA_T    0.01
#define HORIZON_T   DELTA_T * N

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/**
 * Given a current point and a sampled trajectory, compute a suitable 
 * update for the trajectory. Assume the dynamics are those of the
 * main robot.
 */
miam::trajectory::TrajectoryPoint miam_pp::solve_MPC_problem(
    miam::trajectory::SampledTrajectory sampled_trajectory,
    miam::trajectory::TrajectoryPoint current_trajectory_point,
    double current_time
) {
    
    /* Some temporary variables. */
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

    /* Initialize the measurements/reference. */
    for (int j = 0; j < N; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        
        miam::trajectory::TrajectoryPoint current_time_point = sampled_trajectory.getCurrentPoint(current_time + _current_time_offset);
        
        acadoVariables.y[ j * NY ] = current_time_point.position.x;
        acadoVariables.y[ j * NY + 1] = current_time_point.position.y;
        acadoVariables.y[ j * NY + 2] = current_time_point.position.theta;
        acadoVariables.y[ j * NY + 3] = current_time_point.linearVelocity + robotdimensions::wheelSpacing * current_time_point.angularVelocity ;
        acadoVariables.y[ j * NY + 4] = current_time_point.linearVelocity - robotdimensions::wheelSpacing * current_time_point.angularVelocity ;
    }
    
    miam::trajectory::TrajectoryPoint final_time_point = sampled_trajectory.getCurrentPoint(current_time + HORIZON_T);
    acadoVariables.yN[0] = final_time_point.position.x;
    acadoVariables.yN[1] = final_time_point.position.y;
    acadoVariables.yN[2] = final_time_point.position.theta;
    acadoVariables.yN[3] = final_time_point.linearVelocity + robotdimensions::wheelSpacing * final_time_point.angularVelocity;
    acadoVariables.yN[4] = final_time_point.linearVelocity - robotdimensions::wheelSpacing * final_time_point.angularVelocity;
    

    /* MPC: initialize the current state feedback. */
    acadoVariables.x0[0] = current_trajectory_point.position.x;
    acadoVariables.x0[1] = current_trajectory_point.position.y;
    acadoVariables.x0[2] = current_trajectory_point.position.theta;
    acadoVariables.x0[3] = current_trajectory_point.linearVelocity + robotdimensions::wheelSpacing * current_trajectory_point.angularVelocity;
    acadoVariables.x0[4] = current_trajectory_point.linearVelocity - robotdimensions::wheelSpacing * current_trajectory_point.angularVelocity;

	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

    /* Perform the feedback step. */
    acado_feedbackStep( );

    /* Apply the new control immediately to the process, first NU components. */

    if( VERBOSE ) printf("\t KKT Tolerance = %.3e\n\n", acado_getKKT() );

    /* Prepare for the next step. */
    acado_preparationStep();

	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te);

    if (VERBOSE )
    {
        acado_printDifferentialVariables();
        acado_printControlVariables();
    }
    
    /* Get the updated trajectory point at t + n_delay * DELTA_T */
    int n_delay = 1;
    miam::trajectory::TrajectoryPoint updated_current_trajectory_point;
    updated_current_trajectory_point.position.x = acadoVariables.x[n_delay * 5 + 0];
    updated_current_trajectory_point.position.y = acadoVariables.x[n_delay * 5 + 1];
    updated_current_trajectory_point.position.theta = acadoVariables.x[n_delay * 5 + 2];
    updated_current_trajectory_point.linearVelocity = (acadoVariables.x[n_delay * 5 + 3] + acadoVariables.x[n_delay * 5 + 4]) / 2.0;
    updated_current_trajectory_point.angularVelocity = (acadoVariables.x[n_delay * 5 + 3] - acadoVariables.x[n_delay * 5 + 4]) / (robotdimensions::wheelSpacing * 2.0);
    
    return updated_current_trajectory_point;
}
