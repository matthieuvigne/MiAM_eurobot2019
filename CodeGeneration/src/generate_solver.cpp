#include <acado_code_generation.hpp>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>

// Import robotdimensions
#include <MainRobotCode/include/Robot.h>
using namespace robotdimensions;


// Weights of the QP solver
// Along trajectory
double mu_traj = 1 * 50.0;
double mu_theta = 1 * 50.0;
double mu_vlin = 5.0;
double mu_vang = 5.0;


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
    int N = 10;
    
    // Duration of the timestep
    // 10 ms
    double dt = 0.02;
    
    // Final time
    double T = N * dt;

    DifferentialState        x, y, theta;//, v, w    ;
    //~ Control                  vu, wu     ;   
    Control                  v, w     ;   
    DifferentialEquation     f( 0.0, T );

    f << dot(x) == v * cos(theta) ;
    f << dot(y) == v * sin(theta) ;
    f << dot(theta) == w;
    //~ f << dot(v) == vu;
    //~ f << dot(w) == wu;
    
    Function h, hN;
    h << x << y << theta << v << w ;
    hN << x << y << theta ;

    DMatrix W ( h.getDim(), h.getDim() );
    W(0, 0) = mu_traj;
    W(1, 1) = mu_traj;
    W(2, 2) = mu_theta;
    W(3, 3) = mu_vlin;
    W(4, 4) = mu_vang;
    
    DMatrix WN ( hN.getDim(), hN.getDim() );
    WN(0, 0) = 10.0 * mu_traj;
    WN(1, 1) = 10.0 * mu_traj;
    WN(2, 2) = 10.0 * mu_theta;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, T, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    
    
    // Relaxing some constraints
    ocp.subjectTo( -maxWheelSpeed * 1.2 / 1000.0 <= v + (wheelSpacing / 1000.0) * w <= maxWheelSpeed * 1.2 / 1000.0   );     // the control input u,
    //~ ocp.subjectTo( -maxWheelAcceleration * 1.2 / 1000.0 <= vu + (wheelSpacing / 1000.0) * wu <= maxWheelAcceleration * 1.2 / 1000.0   );     // the control input u,

    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        N              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
    mpc.set( GENERATE_TEST_FILE,          NO             );
    mpc.set( GENERATE_MAKE_FILE,          NO             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

//     mpc.set( USE_SINGLE_PRECISION,        YES             );

    if (mpc.exportCode( "MiAM_MPC" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
