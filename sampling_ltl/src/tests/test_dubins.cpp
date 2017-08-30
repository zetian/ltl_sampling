#include "trajectory/dubins_steer.h"

#include <iostream>
#include <cmath>

int main()
{
	std::vector<std::vector<double> > traj_desc(3, std::vector<double>(7, INFINITY));
	std::vector<double> z_0 = {0, 0, 0};
	std::vector<double> z_f = {5, 5, M_PI/2};
	double radius_L = 5;
	double radius_R = 5;
	double min_length;
	DubinsSteer::SteerData dubins_steer_data;
	min_length = DubinsSteer::GetDubinsCurveLength(z_0, z_f, radius_L, radius_R);
	// std::cout << "min length is " << min_length << std::endl;
	dubins_steer_data = DubinsSteer::GetDubinsTrajectoryPointWise(z_0, z_f, radius_L, radius_R);
	std::cout << "Min curve length is " << dubins_steer_data.traj_length << std::endl;

    return 0;
}