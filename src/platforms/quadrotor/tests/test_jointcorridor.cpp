/*
 * test_jointcorridor.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "eigen3/Eigen/Core"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "quad/quad_polyopt.h"

using namespace librav;
using namespace Eigen;

int main(int argc, char* argv[])
{
	uint8_t kf_num = 3;

	QuadPolyOpt opt;

	opt.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);

	// position
	opt.keyframe_x_vals_(0,0) = -0.3;
	opt.keyframe_x_vals_(0,1) = 0.5;
	opt.keyframe_x_vals_(0,2) = 1.05;
	//opt.keyframe_x_vals_(0,3) = 2.5;

	opt.keyframe_y_vals_(0,0) = 1.15;
	opt.keyframe_y_vals_(0,1) = 1;
	opt.keyframe_y_vals_(0,2) = 1.5;
	//opt.keyframe_y_vals_(0,3) = 1.2;

	opt.keyframe_z_vals_(0,0) = 0;
	opt.keyframe_z_vals_(0,1) = 0.2;
	opt.keyframe_z_vals_(0,2) = 0.65;
	//opt.keyframe_z_vals_(0,3) = 1.0;

	// velocity
	opt.keyframe_x_vals_(1,0) = 0;
	opt.keyframe_x_vals_(1,1) = std::numeric_limits<float>::infinity();
	opt.keyframe_x_vals_(1,2) = 0;//std::numeric_limits<float>::infinity();
	//opt.keyframe_x_vals_(1,3) = 0.2;

	opt.keyframe_y_vals_(1,0) = 0;
	opt.keyframe_y_vals_(1,1) = std::numeric_limits<float>::infinity();
	opt.keyframe_y_vals_(1,2) = 0;//std::numeric_limits<float>::infinity();
	//opt.keyframe_y_vals_(1,3) = 0.15;

	opt.keyframe_z_vals_(1,0) = 0;
	opt.keyframe_z_vals_(1,1) = std::numeric_limits<float>::infinity();;
	opt.keyframe_z_vals_(1,2) = 0;
	//opt.keyframe_z_vals_(1,3) = 0.3;

	// acceleration
	opt.keyframe_x_vals_(2,0) = 0;
	opt.keyframe_x_vals_(2,1) = std::numeric_limits<float>::infinity();
	opt.keyframe_x_vals_(2,2) = 0;//std::numeric_limits<float>::infinity();
	//opt.keyframe_x_vals_(1,3) = 0.2;

	opt.keyframe_y_vals_(2,0) = 0;
	opt.keyframe_y_vals_(2,1) = std::numeric_limits<float>::infinity();
	opt.keyframe_y_vals_(2,2) = 0;//std::numeric_limits<float>::infinity();
	//opt.keyframe_y_vals_(1,3) = 0.15;

	opt.keyframe_z_vals_(2,0) = 0;
	opt.keyframe_z_vals_(2,1) = std::numeric_limits<float>::infinity();;
	opt.keyframe_z_vals_(2,2) = 0;
	//opt.keyframe_z_vals_(1,3) = 0.3;

	// yaw
	opt.keyframe_yaw_vals_(0,0) = 0;
	opt.keyframe_yaw_vals_(0,1) = 0;//M_PI/18.0;
	opt.keyframe_yaw_vals_(0,2) = 0;//M_PI/18.0*1.5;
	//opt.keyframe_yaw_vals_(0,3) = 0;//M_PI/18.0*2.0;

	opt.keyframe_ts_(0,0) = 0;
	opt.keyframe_ts_(0,1) = 1.2;
	opt.keyframe_ts_(0,2) = 3;
	//opt.keyframe_ts_(0,3) = 4.5;

	opt.OptimizeFlatTrajWithCorridorJoint();

	//opt.flat_traj_.print();

	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_lcm_msgs::PolynomialCurve_t poly_msg;
	//poly_msg = opt.flat_traj_.GenerateNonDimPolyCurveLCMMsg();

	poly_msg.seg_num = opt.flat_traj_.traj_segs_.size();
	for(auto& seg : opt.flat_traj_.traj_segs_)
	{
		srcl_lcm_msgs::PolyCurveSegment_t seg_msg;

		seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
		seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
		seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
		for(auto& coeff:seg.seg_x.param_.coeffs)
			seg_msg.coeffs_x.push_back(coeff);
		for(auto& coeff:seg.seg_y.param_.coeffs)
			seg_msg.coeffs_y.push_back(coeff);
		for(auto& coeff:seg.seg_z.param_.coeffs)
			seg_msg.coeffs_z.push_back(coeff);

		seg_msg.t_start = 0;
		seg_msg.t_end = 1.0;

		poly_msg.segments.push_back(seg_msg);
	}

	lcm->publish("quad_planner/polynomial_curve", &poly_msg);

	std::cout << "traj sent to lcm" << std::endl;
}

