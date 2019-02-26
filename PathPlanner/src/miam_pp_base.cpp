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


SampledTrajectory miam_pp::get_init_trajectory_from_waypoint_list(
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
    
    double reference_time_horizon = trajectory::distance(first_position, last_position) / NOMINAL_SPEED;
    cout << "Reference time horizon: " << reference_time_horizon << endl;
    
    
    // Resample trajectory points to match MAX_OCP_TIMESTEP * NOMINAL_SPEED
    std::vector<TrajectoryPoint > resampled_trajectory_vector_list;
    
    double t_to_vref = (maxWheelSpeed-first_trajectory_point.linearVelocity) / maxWheelAcceleration;
    double t_from_vref = (maxWheelSpeed-last_trajectory_point.linearVelocity) / maxWheelAcceleration;
    
    cout << "t_to_vref " << t_to_vref << endl;
    cout << "t_from_vref " << t_from_vref << endl;
    
    // First trajectory point
    resampled_trajectory_vector_list.push_back(first_trajectory_point);
    
    for (int i=0; i<waypoint_list.size()-1; i++) 
    {
        RobotPosition current_waypoint = waypoint_list[i];
        RobotPosition vector_between_waypoints = waypoint_list[i+1] - waypoint_list[i];
        
        double distance_between_waypoints = vector_between_waypoints.norm();
        
        int ndiv = std::floor(distance_between_waypoints / (NOMINAL_SPEED * MAX_OCP_TIMESTEP));    
        
        for (int j=0; j<ndiv+1; j++) 
        {
            if (i == 0 & j == 0) continue;
            
            double prop = (j+1.0) / (ndiv+0.0);
            double current_time = i * reference_time_horizon / (waypoint_list.size()-1) + j * distance_between_waypoints / NOMINAL_SPEED / ndiv;    
            
            cout << "current_time " << current_time << endl;
            
            TrajectoryPoint tp_;
            tp_.position = current_waypoint + prop * vector_between_waypoints;
            
            //~ cout << tp_.position << endl;
            
            if (reference_time_horizon - current_time < t_from_vref)
            {
                tp_.linearVelocity = ((current_time - t_from_vref) * maxWheelSpeed + (reference_time_horizon - current_time) * last_trajectory_point.linearVelocity) / t_from_vref / wheelRadius; 
                tp_.linearAcceleration = -maxWheelAcceleration;
                tp_.angularVelocity = 0.0; 
                tp_.linearAcceleration = 0.0;
            }
            else if (current_time < t_to_vref)
            {
                tp_.linearVelocity = (current_time * first_trajectory_point.linearVelocity + (t_to_vref - current_time) * maxWheelSpeed) / t_to_vref / wheelRadius; 
                tp_.linearAcceleration = maxWheelAcceleration;
                tp_.angularVelocity = 0.0;
                tp_.linearAcceleration = 0.0;
            }
            else
            {
                tp_.linearVelocity = maxWheelSpeed / wheelRadius; 
                tp_.linearAcceleration = 0.0;
                tp_.angularVelocity = 0.0;
                tp_.linearAcceleration = 0.0;
            }
            
            resampled_trajectory_vector_list.push_back(tp_);
            
            
        }

    }
    
    // Last trajectory point
    resampled_trajectory_vector_list.push_back(last_trajectory_point);
    
    
    for (int i=0; i<resampled_trajectory_vector_list.size(); i++) {
        TrajectoryPoint tmp = resampled_trajectory_vector_list[i];
        cout << tmp.position << " vlin=" << tmp.linearVelocity << ", wlin=" << tmp.linearAcceleration << ", vang=" << tmp.angularVelocity << ", wang=" << tmp.angularAcceleration << endl;
    }
    
    
        //~ if (N - j < n_timesteps_from_vref) 
        //~ {            
            //~ x_init(j, 3) = j * maxWheelSpeed / N / wheelRadius;
            //~ x_init(j, 4) = j * maxWheelSpeed / N / wheelRadius;
            //~ u_init(j, 0) = maxWheelAcceleration / wheelRadius;
            //~ u_init(j, 1) = maxWheelAcceleration / wheelRadius;
        //~ }
        //~ else if (j < n_timesteps_to_vref)
        //~ {
            //~ x_init(j, 3) = (N-j) * maxWheelSpeed / N / wheelRadius;
            //~ x_init(j, 4) = (N-j) * maxWheelSpeed / N / wheelRadius;
            //~ u_init(j, 0) = -maxWheelAcceleration / wheelRadius;
            //~ u_init(j, 1) = -maxWheelAcceleration / wheelRadius;
        //~ }
        //~ else
        //~ {
            //~ x_init(j, 3) = maxWheelSpeed / wheelRadius;
            //~ x_init(j, 4) = maxWheelSpeed / wheelRadius;
            //~ u_init(j, 0) = 0.0;
            //~ u_init(j, 1) = 0.0;
        //~ }
    
    // N is the number of intervals
    int N = resampled_trajectory_vector_list.size()-1;
    
    //cout << "Resampled waypoints:" << endl;
    //for (RobotPosition _rp : resampled_trajectory_vector_list) 
    //{
        //cout << _rp << endl;
    //}
    
    cout << "Number of resampled waypoints: " << N+1 << endl;
    
    SampledTrajectory res(resampled_trajectory_vector_list, reference_time_horizon);
    cout << "foo" << endl;
    cout << res.getUnderlyingPoints().size() << endl;
    cout << "bar" << endl;
    
    return res;
}


SampledTrajectory miam_pp::get_planned_trajectory_main_robot(
    SampledTrajectory& init_trajectory,
    bool plot,
    bool verbose,
    bool vlin_free_at_end
) 
{
    cout << "get_planned_trajectory_main_robot" << endl;
    
    TrajectoryPoint first_trajectory_point = init_trajectory.getCurrentPoint(0);
    TrajectoryPoint last_trajectory_point = init_trajectory.getEndPoint();
    
    double reference_time_horizon = init_trajectory.getDuration();
    
    
    // Solve OCP using the resampled waypoints
    // using ACADO
    
    // TODO use OCP_N_TIMESTEPS
    
    USING_NAMESPACE_ACADO
    
    int N = init_trajectory.getDuration() / MAX_OCP_TIMESTEP;
    double timestep = reference_time_horizon / N;
    
    // Initialization
    Grid timeGrid (0.0, reference_time_horizon, N+1);
    VariablesGrid x_init(5, timeGrid);
	VariablesGrid u_init(2, timeGrid);
    VariablesGrid p_init(1, timeGrid );
    
    for (int j = 0; j < N+1; j++) {
        
        TrajectoryPoint tp_ = init_trajectory.getCurrentPoint(j * timestep);
        
        x_init(j, 0) = tp_.position.x / 1000.0;
        x_init(j, 1) = tp_.position.y / 1000.0;
        x_init(j, 2) = tp_.position.theta;
        
        // Initialize wheel speed
        {
            WheelSpeed ws_ = drivetrain_kinematics.inverseKinematics(
                BaseSpeed(
                    tp_.linearVelocity, 
                    tp_.angularVelocity
                    )
                );
            x_init(j, 3) = ws_.right;
            x_init(j, 4) = ws_.left;
        }
        
        // Initialize wheel acceleration
        {
            WheelSpeed ws_ = drivetrain_kinematics.inverseKinematics(
                BaseSpeed(
                    tp_.linearAcceleration, 
                    tp_.angularAcceleration
                    )
                );
            u_init(j, 0) = ws_.right;
            u_init(j, 1) = ws_.left;
        }
    }
    p_init(0, 0) = reference_time_horizon;
    
    //std::cout << "Initial values:" << std::endl;
	//x_init.print();
	//u_init.print();

    DifferentialState        x, y, theta, vr, vl    ;
    Control                  wr, wl     ;    
    Parameter                T          ;     
    DifferentialEquation     f( 0.0, T );
    
    OCP ocp( 0.0, T, N );
    ocp.minimizeMayerTerm( T );
    //ocp.minimizeLagrangeTerm( wl*wl );
    //ocp.minimizeLagrangeTerm( wr*wr );
    
    //~ double vlin_normfactor = wheelRadius / 2.0 / 1000.0;
    //~ double vang_normfactor = wheelRadius / 2.0 / wheelSpacing;
    
    f << dot(x) == ((vr + vl) * wheelRadius / 2.0) * cos(theta) / 1000.0;
    f << dot(y) == ((vr + vl) * wheelRadius / 2.0) * sin(theta) / 1000.0;
    f << dot(theta) == ((vr - vl) * wheelRadius / 2.0) / wheelSpacing;
    f << dot(vr) == wr;
    f << dot(vl) == wl;
    
    // First position
    BaseSpeed first_trajectory_point_basespeed(
        first_trajectory_point.linearVelocity,
        first_trajectory_point.angularVelocity
        );
    WheelSpeed first_trajectory_point_wheelspeed =
        drivetrain_kinematics.inverseKinematics(first_trajectory_point_basespeed);

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x ==  first_trajectory_point.position.x / 1000.0 );
    ocp.subjectTo( AT_START, y ==  first_trajectory_point.position.y / 1000.0 );
    ocp.subjectTo( AT_START, theta ==  first_trajectory_point.position.theta );
    ocp.subjectTo( AT_START, vr ==  first_trajectory_point_wheelspeed.right );
    ocp.subjectTo( AT_START, vl == first_trajectory_point_wheelspeed.left );
    
    // Last position
    ocp.subjectTo( AT_END  , x ==  last_trajectory_point.position.x / 1000.0 );
    ocp.subjectTo( AT_END  , y ==  last_trajectory_point.position.y / 1000.0 );
    ocp.subjectTo( AT_END  , theta ==  last_trajectory_point.position.theta );
    
    if (vlin_free_at_end) 
    {
        ocp.subjectTo( AT_END  , vl - vr ==  0 );
    }
    else
    {
        
        BaseSpeed last_trajectory_point_basespeed(
        last_trajectory_point.linearVelocity,
        last_trajectory_point.angularVelocity
        );
        WheelSpeed last_trajectory_point_wheelspeed =
            drivetrain_kinematics.inverseKinematics(
                last_trajectory_point_basespeed
            );
        
        if (verbose)
        {
            cout << last_trajectory_point_wheelspeed.right << " " << last_trajectory_point_wheelspeed.left << endl;
        }
        
        ocp.subjectTo( AT_END  , vr ==  last_trajectory_point_wheelspeed.right );
        ocp.subjectTo( AT_END  , vl ==  last_trajectory_point_wheelspeed.left );
    }
    ocp.subjectTo( -maxWheelSpeed / wheelRadius <= vr <= maxWheelSpeed / wheelRadius );     // the control input u,
    ocp.subjectTo( -maxWheelSpeed / wheelRadius <= vl <= maxWheelSpeed / wheelRadius );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration / wheelRadius <= wr <= maxWheelAcceleration / wheelRadius );     // the control input u,
    ocp.subjectTo( -maxWheelAcceleration / wheelRadius <= wl <= maxWheelAcceleration / wheelRadius );     // the control input u,
    ocp.subjectTo(  0.0 <= T <= 3.0 * reference_time_horizon );
    
    OptimizationAlgorithm algorithm(ocp);
    
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
        window.addSubplot( vl, "vl" );
        window.addSubplot( vr, "vr" );
        window.addSubplot( (vr + vl) * wheelRadius / 2.0 / 1000.0, "vlin" );
        window.addSubplot( (vr - vl) * wheelRadius / 2.0 / wheelSpacing, "vang" );
        window.addSubplot( wl, "wl" );
        window.addSubplot( wr, "wr" );
        algorithm << window;
    }

    
    algorithm.set( INTEGRATOR_TYPE      , INT_RK78        );
    algorithm.set( INTEGRATOR_TOLERANCE , 1e-8            );
    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-8            );
    
    algorithm.initializeDifferentialStates(x_init);
	algorithm.initializeControls(u_init);
	algorithm.initializeParameters(p_init);
    
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
    
    std::vector<TrajectoryPoint > output_trajectory;
    
    for (int i=0; i<N+1; i++) 
    {
        TrajectoryPoint _tp;
        _tp.position.x = statesGrid_final(i, 0) * 1000.0;
        _tp.position.y = statesGrid_final(i, 1) * 1000.0;
        _tp.position.theta = statesGrid_final(i, 2);
        
        WheelSpeed wheel_speed_position(
            statesGrid_final(i, 3),
            statesGrid_final(i, 4)
            );
        BaseSpeed base_speed_position =
            drivetrain_kinematics.forwardKinematics(wheel_speed_position);
        
        _tp.linearVelocity = base_speed_position.linear ;
        _tp.angularVelocity = base_speed_position.angular ;
        output_trajectory.push_back(_tp);
    }
    
    //cout << "Sanity check: " << endl;
    //cout << output_trajectory.front().position << " v= " << output_trajectory.front().linearVelocity << " w=" << output_trajectory.front().angularVelocity << endl;
    //cout << output_trajectory_resampled.front().position << " v= " << output_trajectory_resampled.front().linearVelocity << " w=" << output_trajectory_resampled.front().angularVelocity << endl;
    //cout << output_trajectory.back().position << " v= " << output_trajectory.back().linearVelocity << " w=" << output_trajectory.back().angularVelocity << endl;
    //cout << output_trajectory_resampled.back().position << " v= " << output_trajectory_resampled.back().linearVelocity << " w=" << output_trajectory_resampled.back().angularVelocity << endl;
    
    //shared_ptr<trajectory::SampledTrajectory > output(new );
    
    //cout << "final time " << output->getDuration() << endl;
    
    // Reset static ACADO counters
    Control dummy1;
    DifferentialState dummy2;
    Parameter dummy3;

    dummy1.clearStaticCounters();
    dummy2.clearStaticCounters();
    dummy3.clearStaticCounters();
    
    return SampledTrajectory(output_trajectory, final_time);
}


