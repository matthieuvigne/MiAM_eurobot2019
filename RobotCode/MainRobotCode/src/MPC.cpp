#include <MPC.h>
#include <acado_common.h>
#include <acado_auxiliary_functions.h>
#include <Robot.h>
#include <stdio.h>


using namespace miam;
using namespace miam::trajectory;
using namespace robotdimensions;

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


DrivetrainKinematics dt_kinematics_mpc(
    wheelRadius,
    wheelSpacing,
    encoderWheelRadius,
    encoderWheelSpacing
);


/**
 * Given a current point and a sampled trajectory, compute a suitable 
 * update for the trajectory. Assume the dynamics are those of the
 * main robot.
 */
TrajectoryPoint solve_MPC_problem(
    Trajectory* reference_trajectory,
    TrajectoryPoint current_trajectory_point,
    double current_time
) {
    
    /* Some temporary variables. */
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();
    

    /* Initialize the measurements/reference. */
    for (int j = 0; j < N; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        
        TrajectoryPoint current_time_point = reference_trajectory->getCurrentPoint(current_time + _current_time_offset);
        
        BaseSpeed bs(
            current_time_point.linearVelocity,
            current_time_point.angularVelocity
        );
        WheelSpeed ws =
            dt_kinematics_mpc.inverseKinematics(bs);
        
        acadoVariables.y[ j * NY ] = current_time_point.position.x / 1000.0;
        acadoVariables.y[ j * NY + 1] = current_time_point.position.y / 1000.0;
        acadoVariables.y[ j * NY + 2] = current_time_point.position.theta;
        acadoVariables.y[ j * NY + 3] = ws.right;
        acadoVariables.y[ j * NY + 4] = ws.left ;
    }
    
    {
        TrajectoryPoint final_time_point = reference_trajectory->getCurrentPoint(current_time + HORIZON_T);
        
        BaseSpeed bs(
            final_time_point.linearVelocity,
            final_time_point.angularVelocity
        );
        WheelSpeed ws =
            dt_kinematics_mpc.inverseKinematics(bs);
        
        acadoVariables.yN[0] = final_time_point.position.x / 1000.0;
        acadoVariables.yN[1] = final_time_point.position.y / 1000.0;
        acadoVariables.yN[2] = final_time_point.position.theta;
        acadoVariables.yN[3] = ws.right;
        acadoVariables.yN[4] = ws.left;
    }
    
    
    {
        /* MPC: initialize the current state feedback. */
        BaseSpeed bs(
            current_trajectory_point.linearVelocity,
            current_trajectory_point.angularVelocity
        );
        WheelSpeed ws =
            dt_kinematics_mpc.inverseKinematics(bs);
        
        acadoVariables.x0[0] = current_trajectory_point.position.x / 1000.0;
        acadoVariables.x0[1] = current_trajectory_point.position.y / 1000.0;
        acadoVariables.x0[2] = current_trajectory_point.position.theta;
        acadoVariables.x0[3] = ws.right;
        acadoVariables.x0[4] = ws.left;
    }

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
    TrajectoryPoint updated_current_trajectory_point;
    {
        WheelSpeed ws(
            acadoVariables.x[n_delay * 5 + 3],
            acadoVariables.x[n_delay * 5 + 4]
        );
        BaseSpeed bs =
            dt_kinematics_mpc.forwardKinematics(ws);
        
        updated_current_trajectory_point.position.x = acadoVariables.x[n_delay * 5 + 0] * 1000.0;
        updated_current_trajectory_point.position.y = acadoVariables.x[n_delay * 5 + 1] * 1000.0;
        updated_current_trajectory_point.position.theta = acadoVariables.x[n_delay * 5 + 2];
        updated_current_trajectory_point.linearVelocity = bs.linear;
        updated_current_trajectory_point.angularVelocity = bs.angular;
    }
    
    return updated_current_trajectory_point;
}
