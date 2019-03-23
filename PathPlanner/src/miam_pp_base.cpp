#include <miam_pp_base.hpp>

#include <iostream>
#include <stdexcept>
#include <cmath>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include <memory>

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


using namespace miam;
using namespace miam::trajectory;
using namespace miam_pp;
using namespace std;

double const MAX_OCP_TIMESTEP = 0.02; ///< Max OCP timestep in s
                                      /// (10 ms)

// TODO still unused
// In the future: separate the problem into several smaller problems?
double const OCP_N_TIMESTEPS = 100; ///< Number of OCP timesteps for each
                                    /// problem instance

double const NOMINAL_SPEED = 0.8 * robotdimensions::maxWheelSpeed; ///< Nominal speed
                                                                    /// of the robot


SampledTrajectory miam_pp::get_resampled_trajectory_main_robot(
    WayPointList waypoint_list,
    TrajectoryPoint first_trajectory_point,
    TrajectoryPoint last_trajectory_point
)
{
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
    
    double reference_time_horizon = distance(first_position, last_position) / NOMINAL_SPEED;
    cout << "Reference time horizon: " << reference_time_horizon << endl;
    
    
    // Resample waypoints to match MAX_OCP_TIMESTEP * NOMINAL_SPEED
    TrajectoryVector resampled_waypoint_list;
    for (int i=0; i<waypoint_list.size(); i++) 
    {
        {
            TrajectoryPoint tp_;
            tp_.position = RobotPosition(waypoint_list[i]);
            tp_.linearVelocity = NOMINAL_SPEED;
            tp_.angularVelocity = 0.0;
            resampled_waypoint_list.push_back(tp_);
        }
        
        if (i < waypoint_list.size()-1)
        {
            RobotPosition current_waypoint = waypoint_list[i];
            RobotPosition vector_between_waypoints = waypoint_list[i+1] - waypoint_list[i];
            
            double distance_between_waypoints = vector_between_waypoints.norm();
            
            int ndiv = std::ceil(distance_between_waypoints / (NOMINAL_SPEED * MAX_OCP_TIMESTEP));    
            
            for (int j=0; j<ndiv; j++) 
            {
                double prop = (j+1.0) / (ndiv+1.0);
                
                {
                    TrajectoryPoint tp_;
                    tp_.position = RobotPosition(current_waypoint + prop * vector_between_waypoints);
                    tp_.linearVelocity = NOMINAL_SPEED;
                    tp_.angularVelocity = 0.0;
                    resampled_waypoint_list.push_back(tp_);
                }
            }
        }
    }
    
    return SampledTrajectory(resampled_waypoint_list, reference_time_horizon);
}


SampledTrajectory miam_pp::get_planned_trajectory_main_robot(
    WayPointList waypoint_list,
    TrajectoryPoint first_trajectory_point,
    TrajectoryPoint last_trajectory_point,
    bool plot,
    bool verbose,
    bool vlin_free_at_end
) 
{
    SampledTrajectory init_trajectory = miam_pp::get_resampled_trajectory_main_robot(
        waypoint_list,
        first_trajectory_point,
        last_trajectory_point
    );
    
    int N = init_trajectory.getDuration() / MAX_OCP_TIMESTEP;
    
    
    SampledTrajectory planned_trajectory = miam_pp::get_planned_trajectory_main_robot_from_init(
        init_trajectory,
        first_trajectory_point,
        last_trajectory_point,
        false,
        false,
        vlin_free_at_end,
        20 //max(3, (int)floor(N/2))
    );
    
    SampledTrajectory planned_trajectory_2 = miam_pp::get_planned_trajectory_main_robot_from_init(
        planned_trajectory,
        first_trajectory_point,
        last_trajectory_point,
        plot,
        verbose,
        vlin_free_at_end,
        N
    );
    
    return planned_trajectory_2;
}


SampledTrajectory miam_pp::get_planned_trajectory_main_robot_from_init(
    Trajectory& init_trajectory,
    TrajectoryPoint first_trajectory_point,
    TrajectoryPoint last_trajectory_point,
    bool plot,
    bool verbose,
    bool vlin_free_at_end,
    int N
) 
{
    cout << "get_planned_trajectory_main_robot" << endl;
    
    double reference_time_horizon = init_trajectory.getDuration();
    
    // N is the number of intervals
    if (N == -1)
    {
        N = init_trajectory.getDuration() / MAX_OCP_TIMESTEP;
    }
    
    double reference_timestep = reference_time_horizon / N;
    
    cout << "Init time: " << reference_time_horizon << endl;
    cout << "Reference timestep: " << reference_timestep << endl;
    cout << "Number of resampled waypoints: " << N+1 << endl;
    
    // Solve OCP using the resampled waypoints
    // using ACADO
    
    USING_NAMESPACE_ACADO
    
    DifferentialState        x, y, theta, v, w    ;
    Control                  vu, wu     ; 
    Parameter                T          ;     
    DifferentialEquation     f( 0.0, T );
    
    OCP ocp( 0.0, T, N );
    ocp.minimizeMayerTerm( T );
    
    f << dot(x) == v * cos(theta);
    f << dot(y) == v * sin(theta);
    f << dot(theta) == w;
    f << dot(v) == vu;
    f << dot(w) == wu;
    
    // First position
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x ==  first_trajectory_point.position.x / 1000.0 );
    ocp.subjectTo( AT_START, y ==  first_trajectory_point.position.y / 1000.0 );
    ocp.subjectTo( AT_START, theta ==  first_trajectory_point.position.theta );
    ocp.subjectTo( AT_START, v ==  first_trajectory_point.linearVelocity / 1000.0 );
    ocp.subjectTo( AT_START, w == first_trajectory_point.angularVelocity );
    
    // Last position
    ocp.subjectTo( AT_END  , x ==  last_trajectory_point.position.x / 1000.0 );
    ocp.subjectTo( AT_END  , y ==  last_trajectory_point.position.y / 1000.0 );
    ocp.subjectTo( AT_END  , theta ==  last_trajectory_point.position.theta );
    
    if (!vlin_free_at_end) 
    {
        if (verbose)
        {
            cout << last_trajectory_point.linearVelocity << " " << last_trajectory_point.angularVelocity << endl;
        }
        
        ocp.subjectTo( AT_END  , v ==  last_trajectory_point.linearVelocity / 1000.0 );
        ocp.subjectTo( AT_END  , w ==  last_trajectory_point.angularVelocity );
    }
    ocp.subjectTo( -NOMINAL_SPEED / 1000.0 <= v + (wheelSpacing / 1000.0) * w <= NOMINAL_SPEED / 1000.0   );     // the control input u,
    ocp.subjectTo( -NOMINAL_SPEED / 1000.0 <= v - (wheelSpacing / 1000.0) * w <= NOMINAL_SPEED / 1000.0   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration / 1000.0 <= vu + (wheelSpacing / 1000.0) * wu <= maxWheelAcceleration / 1000.0   );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration / 1000.0 <= vu - (wheelSpacing / 1000.0) * wu <= maxWheelAcceleration / 1000.0   );     // the control input u,
    ocp.subjectTo(  0.0 <= T <= 3.0 * reference_time_horizon );
    
    OptimizationAlgorithm algorithm(ocp);
    
    // Initialization
    Grid timeGrid (0.0, reference_time_horizon, N+1);
    VariablesGrid x_init(5, timeGrid);
    VariablesGrid u_init(2, timeGrid);
    VariablesGrid p_init(1, timeGrid);
    for (int j = 0; j < N+1; j++) {
        TrajectoryPoint tp_ = init_trajectory.getCurrentPoint(j * reference_timestep);
        x_init(j, 0) = tp_.position.x / 1000.0;
        x_init(j, 1) = tp_.position.y / 1000.0;
        x_init(j, 2) = tp_.position.theta;
        x_init(j, 3) = tp_.linearVelocity / 1000.0;
        x_init(j, 4) = tp_.angularVelocity;
    }
    
    for (int j = 0; j < N; j++) {
        u_init(j, 0) = (x_init(j+1, 3) - x_init(j, 3)) / reference_timestep;
        u_init(j, 1) = (x_init(j+1, 4) - x_init(j, 4)) / reference_timestep;
    }
    
    if (verbose)
    {
        cout << x_init << endl;
        cout << u_init << endl;
        cout << p_init << endl;
    }
    
    p_init(0, 0) = reference_time_horizon;
    
    algorithm.initializeDifferentialStates(x_init);
    algorithm.initializeControls(u_init);
    algorithm.initializeParameters(p_init);
    
    if (!verbose) 
    {
        algorithm.set( PRINTLEVEL , NONE );
    }
    
    if (plot) 
    {
        GnuplotWindow window;
        window.addSubplot( x, "x"      );
        window.addSubplot( y, "y"      );
        window.addSubplot( theta, "theta"          );
        window.addSubplot( v, "v" );
        window.addSubplot( w, "w" );
        window.addSubplot( vu, "vu" );
        window.addSubplot( wu, "wu" );
        algorithm << window;
    }

    algorithm.set( INTEGRATOR_TYPE      , INT_RK4        );
    algorithm.set( INTEGRATOR_TOLERANCE , 1e-6            );
    algorithm.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-8            );
    
    algorithm.solve();
    
    VariablesGrid timeGrid_final;
    algorithm.getParameters(timeGrid_final);
    
    VariablesGrid controlsGrid_final;
    algorithm.getControls(controlsGrid_final);
    
    VariablesGrid statesGrid_final;
    algorithm.getDifferentialStates(statesGrid_final);
    
    double final_time = timeGrid_final(N, 0);
    
    if (verbose)
    {
        cout << "Final time: " << final_time << endl;
    }
    
    TrajectoryVector output_trajectory;
    
    for (int i=0; i<N+1; i++) 
    {
        TrajectoryPoint _tp;
        _tp.position.x = statesGrid_final(i, 0) * 1000.0;
        _tp.position.y = statesGrid_final(i, 1) * 1000.0;
        _tp.position.theta = statesGrid_final(i, 2);
        _tp.linearVelocity = statesGrid_final(i, 3) * 1000.0;
        _tp.angularVelocity = statesGrid_final(i, 4);
        output_trajectory.push_back(_tp);
    }
    
    // Reset static ACADO counters
    Control dummy1;
    DifferentialState dummy2;
    Parameter dummy3;

    dummy1.clearStaticCounters();
    dummy2.clearStaticCounters();
    dummy3.clearStaticCounters();
    
    return SampledTrajectory(output_trajectory, final_time);
}


