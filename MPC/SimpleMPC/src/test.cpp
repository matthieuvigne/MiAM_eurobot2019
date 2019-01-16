#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState        x,y,theta, v ;     // the differential states
    Control                  u, a     ;     // the control input u
    Parameter                T          ;     // the time horizon T
    DifferentialEquation     f( 0.0, T );     // the differential equation

//  -------------------------------------

	// Number of time steps
	int N = 100;

    OCP ocp( 0.0, T, N );                        // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T );               // the time T should be optimized

    f << dot(x) == v * cos(theta);                         // an implementation
    f << dot(y) == v * sin(theta);                         // an implementation
    f << dot(theta) == u;                 // for the rocket.
    f << dot(v) == a;                 // for the rocket.

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, x ==  0.0 );     // the initial values for s,
    ocp.subjectTo( AT_START, y ==  0.0 );     // v,
    ocp.subjectTo( AT_START, theta ==  0.0 );     // and m,
    ocp.subjectTo( AT_START, v ==  0.1 );     // and m,

    ocp.subjectTo( AT_END  , x == 10.0 );     // the terminal constraints for s
    ocp.subjectTo( AT_END  , y ==  1.0 );     // and v,
    ocp.subjectTo( AT_END  , theta ==  1.5 );     // and v,
    ocp.subjectTo( AT_END  , v ==  0.0 );     // and v,

    //ocp.subjectTo( -1.0 <= v <=  1.0   );     // as well as the bounds on v
    ocp.subjectTo( -1.1 <= u <=  1.1   );     // the control input u,
    ocp.subjectTo( -1.1 <= a <=  1.1   );     // the control input u,
    ocp.subjectTo(  5.0 <= T <= 100.0   );     // and the time horizon T.
//  -------------------------------------

    GnuplotWindow window;
        window.addSubplot( x, "x"      );
        window.addSubplot( y, "y"      );
        window.addSubplot( theta, "theta"          );
        window.addSubplot( v, "v" );
        window.addSubplot( u, "u" );
        window.addSubplot( a, "a" );
	
    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    algorithm << window;
    algorithm.solve();                        // solves the problem.

	
	algorithm.getDifferentialStates("outputs/states.txt"    );
    algorithm.getParameters        ("outputs/parameters.txt");
    algorithm.getControls          ("outputs/controls.txt"  );
	
	
    return 0;
}
