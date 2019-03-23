#include <iostream>
#include <chrono>
#include <sstream>
#include <string>
#include <fstream>

#include <MiAMEurobot/trajectory/DrivetrainKinematics.h>
#include <MiAMEurobot/io/IOTrajectory.h>
#include "MainRobotCode/include/MPC.h"

// Import robotdimensions
#include <MainRobotCode/include/Robot.h>

using namespace std;
using namespace miam;
using namespace miam::trajectory;

using namespace robotdimensions;


DrivetrainKinematics drivetrain_kinematics(
    wheelRadius,
    wheelSpacing,
    encoderWheelRadius,
    encoderWheelSpacing
);


void appendToStream(
    stringstream& stream,
    double t,
    TrajectoryPoint tp,
    string type
    ) 
{
    
    stream << t << ";" 
        << tp.position.x << ";" 
        << tp.position.y << ";" 
        << tp.position.theta << ";"
        << tp.linearVelocity << ";"
        << tp.angularVelocity << ";"
        << type << endl;
    
};


int main() {
    
    SampledTrajectory sampled_trajectory =
		io::readSampledTrajectoryFromFile("curved_trajectory");

	cout << "Duration: " << sampled_trajectory.getDuration() << endl;
	for (TrajectoryPoint _tp : sampled_trajectory.getUnderlyingPoints()) 
	{
		cout << _tp.position << ", v=" << _tp.linearVelocity << ", w=" << _tp.angularVelocity << endl;
	}
    
    TrajectoryPoint current_trajectory_point = sampled_trajectory.getCurrentPoint(0.0);
    current_trajectory_point.position.x -= 20;
    current_trajectory_point.position.y -= 10;
    
    
    TrajectoryPoint current_trajectory_point_uncorrected = current_trajectory_point;
    
    cout << "Current point: " << current_trajectory_point.position << endl;
    
    vector<TrajectoryPoint > actual_trajectory;
    
    stringstream output_sstream;
    output_sstream << "t" << ";" 
        << "x" << ";" 
        << "y" << ";" 
        << "theta" << ";"
        << "v" << ";"
        << "w" << ";"
        << "type" << endl;
    
    // Starting simulation
    double current_time = 0.0;
    while(current_time < sampled_trajectory.getDuration() + 1.0)
    {
        
        cout << "Cur time: " << current_time << endl;
        
        TrajectoryPoint actual_tp;
        actual_tp = current_trajectory_point;
        actual_trajectory.push_back(actual_tp);
        
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        
        // Compute control
        TrajectoryPoint ctp = MPCsolver::solve_MPC_problem(
            &sampled_trajectory,
            current_trajectory_point,
            max(0.0, current_time)
        );
        
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
        
        TrajectoryPoint refctp = sampled_trajectory.getCurrentPoint(current_time);
        
        cout << "CUR " << current_trajectory_point.linearVelocity << ", " << current_trajectory_point.angularVelocity << ", " << current_trajectory_point.position << endl;
        cout << "MPC " << ctp.linearVelocity << ", " << ctp.angularVelocity << ", " << ctp.position << endl;
        cout << "REF " << refctp.linearVelocity << ", " << refctp.angularVelocity << ", " << refctp.position << endl;
        
        
        appendToStream(
            output_sstream,
            current_time,
            current_trajectory_point,
            "CUR"
        );
        appendToStream(
            output_sstream,
            current_time,
            current_trajectory_point_uncorrected,
            "UNC"
        );
        appendToStream(
            output_sstream,
            current_time,
            refctp,
            "REF"
        );
        
        
        // Feedback
        //~ cout << "Reference position " << refctp.position << endl;
        cout << "Position before " << current_trajectory_point.position << endl; 
        drivetrain_kinematics.integratePosition(
            drivetrain_kinematics.inverseKinematics(
                BaseSpeed(ctp.linearVelocity * 0.01, ctp.angularVelocity * 0.01)
                ), 
            current_trajectory_point.position, 
            false
            );
        drivetrain_kinematics.integratePosition(
            drivetrain_kinematics.inverseKinematics(
                BaseSpeed(refctp.linearVelocity * 0.01, refctp.angularVelocity * 0.01)
                ), 
            current_trajectory_point_uncorrected.position, 
            false
            );
        current_trajectory_point_uncorrected.linearVelocity = refctp.linearVelocity;
        current_trajectory_point_uncorrected.angularVelocity = refctp.angularVelocity;
        
        cout << "Position after " << current_trajectory_point.position << endl;
        cout << "Uncorrected position after " << current_trajectory_point_uncorrected.position << endl;
        current_trajectory_point.linearVelocity = ctp.linearVelocity;
        current_trajectory_point.angularVelocity = ctp.angularVelocity;
        
        // Perturbation
        current_trajectory_point.position.x -= 10* cos(current_trajectory_point.position.theta + 0.05) * 0.01 * (current_time > 0.5 & current_time < 3.0 ? 1 : 0);
        current_trajectory_point.position.y -= 10* sin(current_trajectory_point.position.theta + 0.05) * 0.01 * (current_time > 0.5 & current_time < 3.0 ? 1 : 0);
        current_trajectory_point_uncorrected.position.x -= 10* cos(current_trajectory_point_uncorrected.position.theta + 0.05) * 0.01 * (current_time > 0.5 & current_time < 3.0 ? 1 : 0);
        current_trajectory_point_uncorrected.position.y -= 10* sin(current_trajectory_point_uncorrected.position.theta + 0.05) * 0.01 * (current_time > 0.5 & current_time < 3.0 ? 1 : 0);
        
        // Increment time
        current_time += 0.01;
    }
    
    SampledTrajectory actual_sampled_trajectory(actual_trajectory, sampled_trajectory.getDuration());
    io::writeSampledTrajectoryToFile(actual_sampled_trajectory, "actual_tr");
    
    // Write output
    std::ofstream ofs ("output.csv", std::ofstream::out);

    ofs << output_sstream.str();

    ofs.close();
    
    
    return 0;
}
