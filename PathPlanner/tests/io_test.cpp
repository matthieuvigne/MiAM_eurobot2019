#include <MiAMEurobot/io/IOTrajectory.h>
#include <iostream>

using namespace std;
using namespace miam;

int main() 
{
	trajectory::SampledTrajectory sampled_trajectory =
		io::readSampledTrajectoryFromFile("test_sample_trajectory");

	cout << "Duration: " << sampled_trajectory.getDuration() << endl;
	for (trajectory::TrajectoryPoint _tp : sampled_trajectory.getUnderlyingPoints()) 
	{
		cout << _tp.position << ", v=" << _tp.linearVelocity << ", w=" << _tp.angularVelocity << endl;
	}
}
