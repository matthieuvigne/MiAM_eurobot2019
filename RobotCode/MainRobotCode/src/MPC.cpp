#include <MPC.h>
#include <acado_common.h>
#include <acado_auxiliary_functions.h>
#include <Robot.h>
#include <stdio.h>


using namespace miam;
using namespace miam::trajectory;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */


/*
 * Parameters with which the custom solver was compiled
 */
#define DELTA_T    0.02
#define HORIZON_T   DELTA_T * N

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


void initialize_MPC_problem(
    Trajectory* reference_trajectory,
    double current_time
) {
    /* Initialize the solver. */
    acado_initializeSolver();
    
    for (int j = 0; j < N+1; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        TrajectoryPoint current_time_point = reference_trajectory->getCurrentPoint(current_time + _current_time_offset);
        
        /* Initialize the states. */        
        acadoVariables.x[ j * NX ] = current_time_point.position.x / 1000.0;
        acadoVariables.x[ j * NX + 1] = current_time_point.position.y / 1000.0;
        acadoVariables.x[ j * NX + 2] = current_time_point.position.theta;
    
        /* Initialize the controls. */
        acadoVariables.u[ j * NU ] = current_time_point.linearVelocity / 1000.0;
        acadoVariables.u[ j * NU + 1] = current_time_point.angularVelocity;
    }
    
    // End control: duplicate the preceding control
    acadoVariables.u[ (N-1) * NU ] = acadoVariables.u[ (N-2) * NU ];
    acadoVariables.u[ (N-1) * NU + 1] = acadoVariables.u[ (N-2) * NU + 1 ];
}


TrajectoryPoint solve_MPC_problem(
    Trajectory* reference_trajectory,
    TrajectoryPoint current_trajectory_point,
    double current_time
) {
    
    /* Some temporary variables. */
    acado_timer t;

    // Initial state
    acadoVariables.x0[0] = current_trajectory_point.position.x / 1000.0;
    acadoVariables.x0[1] = current_trajectory_point.position.y / 1000.0;
    acadoVariables.x0[2] = current_trajectory_point.position.theta;
    
    // Initial state
    acadoVariables.x[0] = current_trajectory_point.position.x / 1000.0;
    acadoVariables.x[1] = current_trajectory_point.position.y / 1000.0;
    acadoVariables.x[2] = current_trajectory_point.position.theta;
    
    /* Initialize the measurements/reference. */
    for (int j = 0; j < N; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        
        TrajectoryPoint current_time_point = reference_trajectory->getCurrentPoint(current_time + _current_time_offset);
        
        // State
        acadoVariables.y[ j * NY ] = current_time_point.position.x / 1000.0;
        acadoVariables.y[ j * NY + 1] = current_time_point.position.y / 1000.0;
        acadoVariables.y[ j * NY + 2] = current_time_point.position.theta;
        
        // Control
        acadoVariables.y[ j * NY + 3] = current_time_point.linearVelocity / 1000.0;
        acadoVariables.y[ j * NY + 4] = current_time_point.angularVelocity ;
    }
    

    /* Initialize the final point. */
    
    TrajectoryPoint final_time_point = reference_trajectory->getCurrentPoint(current_time + HORIZON_T);
    
    // State
    acadoVariables.yN[0] = final_time_point.position.x / 1000.0;
    acadoVariables.yN[1] = final_time_point.position.y / 1000.0;
    acadoVariables.yN[2] = final_time_point.position.theta;

    if( VERBOSE ) acado_printHeader();

    /* Prepare step */
    acado_preparationStep();

    /* Get the time before start of the loop. */
    acado_tic( &t );

    /* Perform the feedback step. */
    acado_feedbackStep( );

    /* Apply the new control immediately to the process, first NU components. */

    if( VERBOSE ) printf("\t KKT Tolerance = %.3e\n", acado_getKKT() );

    /* Read the elapsed time. */
    real_t te = acado_toc( &t );

    if( VERBOSE ) printf("Average time of one real-time iteration:   %.3g microseconds\n", 1e6 * te);

    //~ if (VERBOSE )
    //~ {
        //~ acado_printDifferentialVariables();
        //~ acado_printControlVariables();
    //~ }
    
    if( VERBOSE ) {
        for (int i=0; i<NX; i++)
        {
            std::cout << acadoVariables.x[i] << " ";
        }
        std::cout << std::endl;
    }
    
    /* Get the updated trajectory point at t + n_delay * DELTA_T */
    int n_delay = 0;
    
    TrajectoryPoint updated_current_trajectory_point;
    
    // States
    updated_current_trajectory_point.position.x = acadoVariables.x[n_delay * 3 + 0] * 1000.0;
    updated_current_trajectory_point.position.y = acadoVariables.x[n_delay * 3 + 1] * 1000.0;
    updated_current_trajectory_point.position.theta = acadoVariables.x[n_delay * 3 + 2];
    
    // Controls
    updated_current_trajectory_point.linearVelocity = acadoVariables.u[n_delay * 2 + 0] * 1000.0;
    updated_current_trajectory_point.angularVelocity = acadoVariables.u[n_delay * 2 + 1];
    
    return updated_current_trajectory_point;
}
