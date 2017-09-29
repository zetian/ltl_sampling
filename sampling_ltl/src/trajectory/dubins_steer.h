/*
 * dubins_steer.h
 *
 *  Created on: 25 Nov 2016
 *      Author: Zetian
 */

#ifndef SRC_TRAJECTORY_DUBINS_STEER_H_
#define SRC_TRAJECTORY_DUBINS_STEER_H_

#include <vector>
// #include <functional>
namespace DubinsSteer {
	typedef struct{
		std::vector<std::vector<double>> traj_point_wise;
		std::vector<double> traj_len_map;
		double traj_length;
		std::vector<double> z_new;
	}SteerData;


	double GetDubinsCurveLength(std::vector<double> z_0, std::vector<double> z_f, double radius_L, double radius_R);
	SteerData GetDubinsTrajectoryPointWise(std::vector<double> z_0, std::vector<double> z_f, double radius_L, double radius_R);
};

#endif /* SRC_TRAJECTORY_DUBINS_STEER_H_ */