#include <acado_code_generation.hpp>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>

// Import robotdimensions
#include <MainRobotCode/include/Robot.h>
using namespace robotdimensions;


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

    f << dot(x) == (vr + vl) * M_PI * wheelRadius * cos(theta) / 1000.0;
    f << dot(y) == (vr + vl) * M_PI * wheelRadius * sin(theta) / 1000.0;
    f << dot(theta) == (vr - vl) * M_PI * wheelRadius / wheelSpacing;
    f << dot(vr) == wr;
    f << dot(vl) == wl;
    
    Function h, hN;
    h << x << y << theta << vr << vl;
    hN << x << y << theta << vr << vl;

    DMatrix W = eye<double>( h.getDim() );
    DMatrix WN = eye<double>( hN.getDim() );
    WN *= 5;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, T, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo( -maxWheelSpeed <= vr <= maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -maxWheelSpeed <= vl <= maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration <= wr <= maxWheelAcceleration   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration <= wl <= maxWheelAcceleration   );     // the control input u,

	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          YES             );
	mpc.set( GENERATE_MAKE_FILE,          YES             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "MiAM_MPC" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
