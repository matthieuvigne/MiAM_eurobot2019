#include "MiAMEurobot/io/IOTrajectory.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <exception>

using namespace std;

namespace miam{
	namespace io{
	
		trajectory::SampledTrajectory readSampledTrajectoryFromFile(string file_name) 
		{
			
			string line;
			ifstream myfile (file_name, ifstream::in);
			
			vector<trajectory::TrajectoryPoint > trajectory_points;
			
			double duration;
			
			if (myfile.is_open())
			{
				
				// First line: read duration
				getline (myfile,line);
				duration = stod(line);
				

				while ( getline (myfile,line) )
				{
					
					// Parse the line
					istringstream iss(line);
					vector<string> results((istream_iterator<string>(iss)),
													 istream_iterator<string>());
					
					trajectory::TrajectoryPoint trajectory_point;
					RobotPosition robot_position;
					robot_position.x = stod(results[0]);
					robot_position.y = stod(results[1]);
					robot_position.theta = stod(results[2]);
					
					trajectory_point.position = robot_position;
					trajectory_point.linearVelocity = stod(results[3]);
					trajectory_point.angularVelocity = stod(results[4]);
					trajectory_points.push_back(trajectory_point);
					
				}
				
				myfile.close();
				
			}
			else
			{
				throw runtime_error("File does not exist: " + file_name);
			}
			
			return trajectory::SampledTrajectory(trajectory_points, duration);
		}
		
		void writeSampledTrajectoryToFile(trajectory::SampledTrajectory trajectory, string file_name) 
		{
			
			ofstream ofs(file_name, ofstream::out);
			
			ofs << trajectory.getDuration() << endl;
			
			for (trajectory::TrajectoryPoint trajectory_point : trajectory.getUnderlyingPoints()) 
			{
				
				ofs << 
					trajectory_point.position.x << " " <<
					trajectory_point.position.y << " " <<
					trajectory_point.position.theta << " " <<
					trajectory_point.linearVelocity << " " <<
					trajectory_point.angularVelocity << " " <<
					endl;
					
			}
			
			ofs.close();
			
		}
	}
}
