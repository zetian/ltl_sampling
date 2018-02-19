/*
 * test_opt.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include "eigen3/Eigen/Core"

#include "polyopt/gurobi_solver/gurobi_polyopt.h"

using namespace librav;
using namespace Eigen;

int main(int argc, char* argv[])
{
	uint32_t r = 4;
	uint32_t N = 2 * r - 1;
	uint8_t kf_num = 4;

	MatrixXf keyframe_vals = MatrixXf::Zero(r, kf_num);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num);

	keyframe_vals(0,0) = -0.15;
	keyframe_vals(0,1) = 0.25;
	keyframe_vals(0,2) = 0.3;
	keyframe_vals(0,3) = 0.35;

	keyframe_vals(1,0) = 0;
	keyframe_vals(1,1) = std::numeric_limits<float>::infinity();
	keyframe_vals(1,2) = std::numeric_limits<float>::infinity();
	keyframe_vals(1,3) = 0;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;
	keyframe_ts(0,2) = 3;
	keyframe_ts(0,3) = 4.5;

	GurobiPolyOpt traj_opt;
	OptResultCurve result;
	result = traj_opt.OptimizeTrajectory(keyframe_vals, keyframe_ts, kf_num, N, r);

//	std::cout << "**************************************" << std::endl;
//	std::cout << "cost: " << result.cost << std::endl;
//	std::cout << "segments: " << std::endl;
//	uint32_t idx = 0;
//	for(auto& seg:result.segments)
//	{
//		std::cout << "seg " << idx << " : " << std::endl;
//		for(auto& coef:seg.coeffs)
//			std::cout << coef << std::endl;
//		std::cout << "start time: " << seg.ts << " , end time: " << seg.te << std::endl;
//		idx++;
//	}
	result.print();
}


