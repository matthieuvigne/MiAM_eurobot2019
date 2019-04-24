#include <MPC.h>
#include <acado_common.h>
#include <acado_auxiliary_functions.h>
#include <Robot.h>
#include <stdio.h>

using namespace std;
using namespace miam;
using namespace miam::trajectory;

//~ #ifdef MIAM_MPC_VELOCITY_CONTROL
//~ #define MIAM_MPC_VELOCITY_CONTROL true
//~ #else
#define MIAM_MPC_VELOCITY_CONTROL false
//~ #endif

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */


/*
 * Parameters with which the custom solver was compiled
 */
#define DELTA_T    0.01
#define HORIZON_T   DELTA_T * N

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* Initialize static variables*/
Trajectory* miam::MPCsolver::_mpc_current_trajectory = nullptr;
bool miam::MPCsolver::_mpc_solver_initialized = false;


void miam::MPCsolver::initialize_MPC_solver() 
{
    /* Initialize the solver. */
    cout << "Initializing solver" << endl;
    acado_initializeSolver();
}    


void miam::MPCsolver::initialize_MPC_problem(
    Trajectory* reference_trajectory,
    double current_time
) {
    /* Initialize the initial guesses. */
    
    for (int j = 0; j < N+1; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        TrajectoryPoint current_time_point = reference_trajectory->getCurrentPoint(current_time + _current_time_offset);
        
        /* Initialize the states. */        
        acadoVariables.x[ j * NX ] = current_time_point.position.x / 1000.0;
        acadoVariables.x[ j * NX + 1] = current_time_point.position.y / 1000.0;
        acadoVariables.x[ j * NX + 2] = current_time_point.position.theta;
    
        /* Initialize the controls. */
#if MIAM_MPC_VELOCITY_CONTROL
        acadoVariables.u[ j * NU ] = current_time_point.linearVelocity / 1000.0;
        acadoVariables.u[ j * NU + 1] = current_time_point.angularVelocity;
#else
        acadoVariables.x[ j * NU + 3] = current_time_point.linearVelocity / 1000.0;
        acadoVariables.x[ j * NU + 4] = current_time_point.angularVelocity;
#endif
    }
    
#if MIAM_MPC_VELOCITY_CONTROL
    // End control: duplicate the preceding control
    acadoVariables.u[ (N-1) * NU ] = acadoVariables.u[ (N-2) * NU ];
    acadoVariables.u[ (N-1) * NU + 1] = acadoVariables.u[ (N-2) * NU + 1 ];
#endif
}


TrajectoryPoint miam::MPCsolver::solve_MPC_problem(
    Trajectory* reference_trajectory,
    TrajectoryPoint current_trajectory_point,
    double current_time
) {
    
    if (!miam::MPCsolver::_mpc_solver_initialized)
    {
        initialize_MPC_solver();
        miam::MPCsolver::_mpc_solver_initialized = true;
    }
    
    if (miam::MPCsolver::_mpc_current_trajectory != reference_trajectory)
    {
        cout << "solve_MPC_problem: new traj detected" << endl;
        initialize_MPC_problem(reference_trajectory, current_time);
        miam::MPCsolver::_mpc_current_trajectory = reference_trajectory;
    }
    
    /* Some temporary variables. */
    acado_timer t;

    // Initial state
    acadoVariables.x0[0] = current_trajectory_point.position.x / 1000.0;
    acadoVariables.x0[1] = current_trajectory_point.position.y / 1000.0;
    acadoVariables.x0[2] = current_trajectory_point.position.theta;
#if !MIAM_MPC_VELOCITY_CONTROL
    acadoVariables.x0[3] = current_trajectory_point.linearVelocity / 1000.0;
    acadoVariables.x0[4] = current_trajectory_point.angularVelocity;
#endif
    
    // Initial state
    acadoVariables.x[0] = current_trajectory_point.position.x / 1000.0;
    acadoVariables.x[1] = current_trajectory_point.position.y / 1000.0;
    acadoVariables.x[2] = current_trajectory_point.position.theta;
#if !MIAM_MPC_VELOCITY_CONTROL
    acadoVariables.x[3] = current_trajectory_point.linearVelocity / 1000.0;
    acadoVariables.x[4] = current_trajectory_point.angularVelocity;
#endif
    
    /* Initialize the measurements/reference. */
    for (int j = 0; j < N; j++) {
        double _current_time_offset = HORIZON_T * j / N;
        
        TrajectoryPoint current_time_point = reference_trajectory->getCurrentPoint(current_time + _current_time_offset);
        
        // State
        acadoVariables.y[ j * NY ] = current_time_point.position.x / 1000.0;
        acadoVariables.y[ j * NY + 1] = current_time_point.position.y / 1000.0;
        acadoVariables.y[ j * NY + 2] = current_time_point.position.theta;
        acadoVariables.y[ j * NY + 3] = current_time_point.linearVelocity / 1000.0;
        acadoVariables.y[ j * NY + 4] = current_time_point.angularVelocity ;
    }
    

    /* Initialize the final point. */
    TrajectoryPoint final_time_point = reference_trajectory->getCurrentPoint(current_time + HORIZON_T);
    
    // State
    acadoVariables.yN[0] = final_time_point.position.x / 1000.0;
    acadoVariables.yN[1] = final_time_point.position.y / 1000.0;
    acadoVariables.yN[2] = final_time_point.position.theta;
    //~ acadoVariables.yN[3] = final_time_point.linearVelocity;
    //~ acadoVariables.yN[4] = final_time_point.angularVelocity;

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
            cout << acadoVariables.x[i] << " ";
        }
        cout << endl;
    }
    
    /* Get the updated trajectory point at t + n_delay * DELTA_T */
    int n_delay = 1;
    
    TrajectoryPoint updated_current_trajectory_point;
    
    // States
    updated_current_trajectory_point.position.x = acadoVariables.x[n_delay * NX + 0] * 1000.0;
    updated_current_trajectory_point.position.y = acadoVariables.x[n_delay * NX + 1] * 1000.0;
    updated_current_trajectory_point.position.theta = acadoVariables.x[n_delay * NX + 2];
    
    // Controls
#if MIAM_MPC_VELOCITY_CONTROL
    updated_current_trajectory_point.linearVelocity = acadoVariables.u[n_delay * NU + 0] * 1000.0;
    updated_current_trajectory_point.angularVelocity = acadoVariables.u[n_delay * NU + 1];
#else
    updated_current_trajectory_point.linearVelocity = acadoVariables.x[n_delay * NX + 3] * 1000.0;
    updated_current_trajectory_point.angularVelocity = acadoVariables.x[n_delay * NX + 4];
#endif
    
    return updated_current_trajectory_point;
}
