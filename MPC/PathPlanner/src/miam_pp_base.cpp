#include <miam_pp_base.hpp>

#include <iostream>
#include <stdexcept>
#include <cmath>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

using namespace miam;
using namespace miam_pp;
using namespace std;

double const MAX_OCP_TIMESTEP = 0.01; ///< Max OCP timestep in s
                                      /// (10 ms)

// TODO still unused
// In the future: separate the problem into several smaller problems?
double const OCP_N_TIMESTEPS = 100; ///< Number of OCP timesteps for each
                                    /// problem instance

double const NOMINAL_SPEED = 0.95 * robotdimensions::maxWheelSpeed; ///< Nominal speed
                                                                    /// of the robot

TrajectoryVector miam_pp::get_planned_trajectory_main_robot(
    WayPointList waypoint_list
) 
{
    cout << "get_planned_trajectory_main_robot" << endl;
    
    if (waypoint_list.size() < 2) 
    {
        throw runtime_error("Specify 2 or more waypoints");
    }
    
    cout << "Provided waypoints:" << endl;
    for (RobotPosition _rp : waypoint_list) 
    {
        cout << _rp << endl;
    }
    
    RobotPosition first_position = waypoint_list.front();
    RobotPosition last_position = waypoint_list.back();
    
    double reference_time_horizon = trajectory::distance(first_position, last_position) / NOMINAL_SPEED;
    cout << "Reference time horizon: " << reference_time_horizon << endl;
    
    
    // Resample waypoints to match MAX_OCP_TIMESTEP * NOMINAL_SPEED
    WayPointList resampled_waypoint_list = WayPointList();
    for (int i=0; i<waypoint_list.size(); i++) 
    {
        resampled_waypoint_list.push_back(RobotPosition(waypoint_list[i]));
        if (i < waypoint_list.size()-1)
        {
            RobotPosition current_waypoint = waypoint_list[i];
            RobotPosition vector_between_waypoints = waypoint_list[i+1] - waypoint_list[i];
            
            double distance_between_waypoints = vector_between_waypoints.norm();
            
            int ndiv = std::ceil(distance_between_waypoints / (NOMINAL_SPEED * MAX_OCP_TIMESTEP));    
            
            for (int j=0; j<ndiv; j++) 
            {
                double prop = (j+1.0) / (ndiv+1.0);
                resampled_waypoint_list.push_back(current_waypoint + prop * vector_between_waypoints);
            }
        }
    }
    
    int N = resampled_waypoint_list.size();
    
    //cout << "Resampled waypoints:" << endl;
    //for (RobotPosition _rp : resampled_waypoint_list) 
    //{
        //cout << _rp << endl;
    //}
    
    cout << "Number of resampled waypoints: " << N << endl;
    
    // Solve OCP using the resampled waypoints
    // using ACADO
    
    // TODO use OCP_N_TIMESTEPS
    
    USING_NAMESPACE_ACADO
    
    
    // Initialization
    Grid timeGrid (0.0, reference_time_horizon, N);
    VariablesGrid x_init(5, timeGrid);
	VariablesGrid u_init(2, timeGrid);
    for (int j = 0; j < N; j++) {
            x_init(j, 0) = resampled_waypoint_list[j].x;
            x_init(j, 1) = resampled_waypoint_list[j].y;
            x_init(j, 2) = resampled_waypoint_list[j].theta;
            x_init(j, 3) = NOMINAL_SPEED;
            x_init(j, 4) = NOMINAL_SPEED;
			
			u_init(j, 0) = 0.0;
			u_init(j, 1) = 0.0;
		}
    
    //std::cout << "Initial values:" << std::endl;
	//x_init.print();
	//u_init.print();

    DifferentialState        x, y, theta, vr, vl    ;
    Control                  wl, wr     ;    
    Parameter                T          ;     
    DifferentialEquation     f( 0.0, T );
    
    OCP ocp( 0.0, T, N );
    ocp.minimizeMayerTerm( T );
    //ocp.minimizeLagrangeTerm( wl*wl );
    //ocp.minimizeLagrangeTerm( wr*wr );

    f << dot(x) == (vl + vr) * cos(theta) / 2.0;
    f << dot(y) == (vl + vr) * sin(theta) / 2.0;
    f << dot(theta) == (vr - vl) / (2.0 * robotdimensions::wheelSpacing);
    f << dot(vr) == wr;
    f << dot(vl) == wl;

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x ==  first_position.x );
    ocp.subjectTo( AT_START, y ==  first_position.y );
    ocp.subjectTo( AT_START, theta ==  first_position.theta );
    ocp.subjectTo( AT_START, vr ==  0.0 );
    ocp.subjectTo( AT_START, vl ==  0.0 );

    ocp.subjectTo( AT_END  , x ==  last_position.x );
    ocp.subjectTo( AT_END  , y ==  last_position.y );
    ocp.subjectTo( AT_END  , theta ==  last_position.theta );
    ocp.subjectTo( AT_END  , vr ==  0.0 );
    ocp.subjectTo( AT_END  , vl ==  0.0 );

    ocp.subjectTo( -robotdimensions::maxWheelSpeed <= vr <= robotdimensions::maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelSpeed <= vl <= robotdimensions::maxWheelSpeed   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelAcceleration <= wr <= robotdimensions::maxWheelAcceleration   );     // the control input u,
    ocp.subjectTo( -robotdimensions::maxWheelAcceleration <= wl <= robotdimensions::maxWheelAcceleration   );     // the control input u,
    ocp.subjectTo(  0.0 <= T <= 3.0 * reference_time_horizon );

    GnuplotWindow window;
    window.addSubplot( x, "x"      );
    window.addSubplot( y, "y"      );
    window.addSubplot( theta, "theta"          );
    window.addSubplot( vl, "vl" );
    window.addSubplot( vr, "vr" );
    window.addSubplot( (vl + vr) / 2.0, "v" );
    window.addSubplot( wl, "wl" );
    window.addSubplot( wr, "wr" );
    
    OptimizationAlgorithm algorithm(ocp);
    algorithm << window;
    
    algorithm.set( INTEGRATOR_TYPE      , INT_RK78        );
    algorithm.set( INTEGRATOR_TOLERANCE , 1e-8            );
    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-8            );
    
    algorithm.initializeDifferentialStates(x_init);
	algorithm.initializeControls(u_init);
    
    algorithm.solve();

    algorithm.getDifferentialStates("outputs/states.txt"    );
    algorithm.getParameters        ("outputs/parameters.txt");
    algorithm.getControls          ("outputs/controls.txt"  );
    
    VariablesGrid timeGrid_final;
    algorithm.getParameters(timeGrid_final);
    
    VariablesGrid controlsGrid_final;
	algorithm.getControls(controlsGrid_final);
	
	VariablesGrid statesGrid_final;
	algorithm.getDifferentialStates(statesGrid_final);
    
    TrajectoryVector output_trajectory;
    
    for (int i=0; i<N+1; i++) 
    {
        trajectory::TrajectoryPoint _tp;
        _tp.position.x = statesGrid_final(i, 0);
        _tp.position.y = statesGrid_final(i, 1);
        _tp.position.theta = statesGrid_final(i, 2);
        _tp.linearVelocity = (statesGrid_final(i, 3) + statesGrid_final(i, 4)) / 2.0;
        _tp.angularVelocity = (statesGrid_final(i, 3) - statesGrid_final(i, 4)) / (2.0 * robotdimensions::wheelSpacing);
        output_trajectory.push_back(_tp);
    }
    
    return output_trajectory;
}


