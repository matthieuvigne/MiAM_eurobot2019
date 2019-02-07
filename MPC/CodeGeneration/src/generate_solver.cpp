//#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

// Define the robot dimensions if Robot.h was not defined
#ifndef ROBOT_H
namespace robotdimensions
	{
		double const wheelRadius = 51.0; ///< Wheel radius, in mm.
		double const wheelSpacing = 106.0; ///< Wheel spacing from robot center, in mm.
		double const encoderWheelRadius = 26.0; ///< Radius of encoder wheels, in mm.
		double const encoderWheelSpacing = 133.0; ///< Encoder wheel spacing from robot center, in mm.

		double const stepSize = 2 * 3.14159265359 / 600.0; ///< Size of a motor step, in rad.

		double const maxWheelSpeed = 600; ///< Maximum wheel speed, in mm/s.
		double const maxWheelAcceleration = 800; ///< Maximum wheel acceleration, in mm/s^2.
}
#endif


int main( )
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
    Control                  wl, wr     ;    
    //Parameter                T          ;     
    DifferentialEquation     f( 0.0, T );
    
    //OCP ocp( 0.0, T, N );
    //ocp.minimizeMayerTerm( T );
    //ocp.minimizeLagrangeTerm( wl*wl );
    //ocp.minimizeLagrangeTerm( wr*wr );

    f << dot(x) == (vl + vr) * cos(theta) / 2.0;
    f << dot(y) == (vl + vr) * sin(theta) / 2.0;
    f << dot(theta) == (vr - vl) / (2.0 * robotdimensions::wheelSpacing);
    f << dot(vr) == wr;
    f << dot(vl) == wl;

    //ocp.subjectTo( f );
    
    Function h, hN;
    h << x << y << theta << vr << vl;
    hN << x << y << theta << vr << vl;

    DMatrix W = eye<double>( h.getDim() );
    DMatrix WN = eye<double>( hN.getDim() );
    WN *= 5;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, T, N+1);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    
    //ocp.subjectTo( AT_START, x ==  first_trajectory_point.x );
    //ocp.subjectTo( AT_START, y ==  first_trajectory_point.y );
    //ocp.subjectTo( AT_START, theta ==  first_trajectory_point.theta );
    //ocp.subjectTo( AT_START, vr ==  0.0 );
    //ocp.subjectTo( AT_START, vl ==  0.0 );

    //ocp.subjectTo( AT_END  , x ==  last_trajectory_point.x );
    //ocp.subjectTo( AT_END  , y ==  last_trajectory_point.y );
    //ocp.subjectTo( AT_END  , theta ==  last_trajectory_point.theta );
    //ocp.subjectTo( AT_END  , vr ==  0.0 );
    //ocp.subjectTo( AT_END  , vl ==  0.0 );

    ocp.subjectTo( -robotdimensions::maxWheelSpeed <= vr <= robotdimensions::maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelSpeed <= vl <= robotdimensions::maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelAcceleration <= wr <= robotdimensions::maxWheelAcceleration   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelAcceleration <= wl <= robotdimensions::maxWheelAcceleration   );     // the control input u,
    //ocp.subjectTo(  0.0 <= T );

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

	if (mpc.exportCode( "getting_started_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
