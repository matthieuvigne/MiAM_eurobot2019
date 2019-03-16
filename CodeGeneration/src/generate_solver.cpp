#include <acado_code_generation.hpp>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>

// Import robotdimensions
#include <MainRobotCode/include/Robot.h>
using namespace robotdimensions;


// Weights of the QP solver
// Along trajectory
double mu_traj = 10.0;
double mu_theta = 2.0;
double mu_vel = 0.01;
//~ double mu_control = 0.3;
// At the end
double mu_end_traj = 10.0;
double mu_end_theta = 2.0;
double mu_end_vel = 0.01;
//~ double mu_end_control = 0.3 * 5;


DrivetrainKinematics drivetrain_kinematics(
    wheelRadius,
    wheelSpacing,
    encoderWheelRadius,
    encoderWheelSpacing
);


int main()
{
    USING_NAMESPACE_ACADO
    
    // Number of time intervals
    int N = 20;
    
    // Duration of the timestep
    // 10 ms
    double dt = 0.01;
    
    // Final time
    double T = N * dt;

    DifferentialState        x, y, theta, vr, vl    ;
    Control                  wr, wl     ;   
    DifferentialEquation     f( 0.0, T );

    f << dot(x) == (vr + vl) * wheelRadius * cos(theta) / 2.0 / 1000.0;
    f << dot(y) == (vr + vl) * wheelRadius * sin(theta) / 2.0 / 1000.0;
    f << dot(theta) == (vr - vl) * wheelRadius / 2.0 / wheelSpacing;
    f << dot(vr) == wr;
    f << dot(vl) == wl;
    
    Function h, hN;
    h << x << y << theta << vr << vl;
    hN << x << y << theta << vr << vl;

    DMatrix W ( h.getDim(), h.getDim() );
    W(0, 0) = mu_traj;
    W(1, 1) = mu_traj;
    W(2, 2) = mu_theta;
    W(3, 3) = mu_vel;
    W(4, 4) = mu_vel;
    
    DMatrix WN ( hN.getDim(), hN.getDim() );
    WN(0, 0) = mu_end_traj;
    WN(1, 1) = mu_end_traj;
    WN(2, 2) = mu_end_theta;
    WN(3, 3) = mu_end_vel;
    WN(4, 4) = mu_end_vel;    

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, T, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    
    
    // Relaxing some constraints
    ocp.subjectTo( -maxWheelSpeed * 1.1 <= vr <= maxWheelSpeed * 1.1   );     // the control input u,
    ocp.subjectTo( -maxWheelSpeed * 1.1 <= vl <= maxWheelSpeed * 1.1   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration * 10 <= wr <= maxWheelAcceleration * 10   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration * 10 <= wl <= maxWheelAcceleration * 10   );     // the control input u,

    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    //~ mpc.set( NUM_INTEGRATOR_STEPS,        N              );
    mpc.set( FIX_INITIAL_STATE,           YES              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
//     mpc.set( HOTSTART_QP,                 YES             );
//     mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
    mpc.set( GENERATE_TEST_FILE,          YES             );
    mpc.set( GENERATE_MAKE_FILE,          YES             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

//     mpc.set( USE_SINGLE_PRECISION,        YES             );

    if (mpc.exportCode( "MiAM_MPC" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
