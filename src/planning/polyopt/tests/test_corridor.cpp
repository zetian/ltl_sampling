/*
 * test_corridor.cpp
 *
 *  Created on: Sep 29, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cstdint>
#include <limits>
#include <cstring>
#include <vector>
#include <memory>

#include "gurobi_c++.h"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_math.h"
#include "polyopt/gurobi_utils.h"
#include "common/poly_curve.h"

using namespace librav;
using namespace Eigen;

int main(int argc, char* argv[])
{
	uint32_t r = 4;
	uint32_t N = 2 * r - 1;

	uint8_t kf_num = 3;

	// init variables
	MatrixXf keyframe_x_vals = MatrixXf::Zero(r, kf_num);
	MatrixXf keyframe_y_vals = MatrixXf::Zero(r, kf_num);
	MatrixXf keyframe_z_vals = MatrixXf::Zero(r, kf_num);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num);

	MatrixXf Q_x = Eigen::MatrixXf::Zero((kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1));
	MatrixXf Q_y = Eigen::MatrixXf::Zero((kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1));
	//MatrixXf Q_z = Eigen::MatrixXf::Zero((kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1));
	MatrixXf Aeq_x = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1));
	MatrixXf Aeq_y = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1));
	//MatrixXf Aeq_z = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1));
	MatrixXf beq_x = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, 1);
	MatrixXf beq_y = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, 1);
	//MatrixXf beq_z = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r, 1);

	MatrixXf Q_joint = Eigen::MatrixXf::Zero((kf_num - 1) * (N + 1) * 2, (kf_num - 1) * (N + 1) * 2);
	MatrixXf Aeq_joint = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r * 2, (kf_num - 1) * (N + 1) * 2);
	MatrixXf beq_joint = Eigen::MatrixXf::Zero((kf_num - 1) * 2 * r * 2, 1);

	int var_size = (N + 1)*(kf_num - 1)*2;

	std::vector<MatrixXf> keyframe_vals;

	// specify values
	keyframe_x_vals(0,0) = -0.3;
	keyframe_x_vals(0,1) = 0.5;
	keyframe_x_vals(0,2) = 1.05;
	//keyframe_x_vals(0,3) = 0.35;

	keyframe_y_vals(0,0) = 1.15;
	keyframe_y_vals(0,1) = 1.0;
	keyframe_y_vals(0,2) = 1.5;
	//eyframe_y_vals(0,3) = 0.45;

	//keyframe_z_vals(0,0) = -0.0;
	//keyframe_z_vals(0,1) = 0.15;
	//keyframe_z_vals(0,2) = 0.2;
	//keyframe_z_vals(0,3) = 0.15;

	keyframe_x_vals(1,1) = std::numeric_limits<float>::infinity();
	keyframe_y_vals(1,1) = std::numeric_limits<float>::infinity();

	keyframe_x_vals(2,1) = std::numeric_limits<float>::infinity();
	keyframe_y_vals(2,1) = std::numeric_limits<float>::infinity();

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;
	keyframe_ts(0,2) = 3;
	//keyframe_ts(0,3) = 4.5;

	int dim = 2;
	keyframe_vals.push_back(keyframe_x_vals);
	keyframe_vals.push_back(keyframe_y_vals);
	//keyframe_vals.push_back(keyframe_z_vals);

	int nc = 20;
	double max_dist = 0.01;

	Eigen::MatrixXf A_cor = MatrixXf::Zero(nc * 2 * dim * (kf_num - 1), (N + 1) * dim * (kf_num - 1));
	Eigen::MatrixXf b_cor = MatrixXf::Zero(nc * 2 * dim * (kf_num - 1), 1);

	PolyOptMath::GetNonDimQMatrices(N,r,kf_num, keyframe_ts, Q_x);
	PolyOptMath::GetNonDimQMatrices(N,r,kf_num, keyframe_ts, Q_y);
	//PolyOptMath::GetNonDimQMatrices(N,r,kf_num, keyframe_ts, Q_z);

	PolyOptMath::GetNonDimEqualityConstrs(N,r,kf_num, keyframe_x_vals, keyframe_ts, Aeq_x, beq_x);
	PolyOptMath::GetNonDimEqualityConstrs(N,r,kf_num, keyframe_y_vals, keyframe_ts, Aeq_y, beq_y);
	//PolyOptMath::GetNonDimEqualityConstrs(N,r,kf_num, keyframe_z_vals, keyframe_ts, Aeq_z, beq_z);

	Q_joint.block(0, 0, (kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1)) = Q_x;
	Q_joint.block((kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1)) = Q_y;
	//Q_joint.block((kf_num - 1) * (N + 1)*2, (kf_num - 1) * (N + 1)*2, (kf_num - 1) * (N + 1), (kf_num - 1) * (N + 1)) = Q_z;

	Aeq_joint.block(0 , 0, (kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1)) = Aeq_x;
	Aeq_joint.block((kf_num - 1) * 2 * r , (kf_num - 1) * (N + 1), (kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1)) = Aeq_y;
	//Aeq_joint.block((kf_num - 1) * 2 * r * 2 , (kf_num - 1) * (N + 1) * 2, (kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1)) = Aeq_z;

	beq_joint.block(0 , 0, (kf_num - 1) * 2 * r, 1) = beq_x;
	beq_joint.block((kf_num - 1) * 2 * r, 0, (kf_num - 1) * 2 * r, 1) = beq_y;
	//beq_joint.block((kf_num - 1) * 2 * r * 2 , 0, (kf_num - 1) * 2 * r, 1) = beq_z;

	PolyOptMath::GetNonDimCorridorConstrs(N, kf_num , nc , max_dist, keyframe_vals, keyframe_ts, A_cor, b_cor);

//	std::cout << "Q_joint: \n" << Q_joint << std::endl;
//	std::cout << "Aeq_joint: \n" << Aeq_joint << std::endl;
//	std::cout << "beq_joint: \n" << beq_joint << std::endl;
//	std::cout << "A_cor: \n" << A_cor << std::endl;
//	std::cout << "b_cor: \n" << b_cor << std::endl;

	try {
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		std::vector<GRBVar> sig;
		sig.resize(var_size);
		for(int i = 0; i < var_size; i++)
		{
			std::string var_name = "sig"+std::to_string(i);
			sig[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}

		// Integrate new variables
		model.update();

		// Set objective
		GRBQuadExpr cost_fun;
		GurobiUtils::GetQuadraticCostFuncExpr(sig, Q_joint, var_size, cost_fun);;
		model.setObjective(cost_fun);

		// Add constraints
		GurobiUtils::AddLinEqualityConstrExpr(sig, Aeq_joint, beq_joint, var_size, var_size, model);
		GurobiUtils::AddLinInequalityConstrExpr(sig, A_cor, b_cor, var_size, nc * 2 * dim * (kf_num - 1), model);

		// Optimize model
		model.optimize();

		std::vector<double> params;
		for(int i = 0; i < var_size; i++)
		{
			if(i != 0 && i%(N+1) == 0)
				std::cout << std::endl;
			std::cout << sig[i].get(GRB_StringAttr_VarName) << " "
			<< sig[i].get(GRB_DoubleAttr_X) << std::endl;

			params.push_back(sig[i].get(GRB_DoubleAttr_X));
		}

		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

		std::vector<std::vector<CurveParameter>> traj;
		traj.reserve(2);
		for(int dim = 0; dim < 2; dim++)
		{
			std::vector<CurveParameter> segs;
			for(int seg = 0; seg < kf_num - 1; seg++)
			{
				CurveParameter seg_param;

				for(int i = 0; i < N + 1; i++) {
					//std::cout << "index: " << dim*(kf_num - 1)*(N + 1) + seg * (N + 1) + i << std::endl;
					seg_param.coeffs.push_back(params[dim*(kf_num - 1)*(N + 1) + seg * (N + 1) + i]);
				}

				seg_param.ts = keyframe_ts(0, seg);
				seg_param.te = keyframe_ts(0, seg+1);

				segs.push_back(seg_param);
			}
			traj.push_back(segs);
		}

		// send data for visualization
		std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

		if(!lcm->good())
		{
			std::cout << "ERROR: Failed to initialize LCM." << std::endl;
			return -1;
		}

		srcl_lcm_msgs::PolynomialCurve_t poly_msg;
		poly_msg.seg_num = 2;
		for(int i = 0; i < 2; i++)
		{
			srcl_lcm_msgs::PolyCurveSegment_t seg_msg;

			seg_msg.coeffsize_x = traj[0][i].coeffs.size();
			seg_msg.coeffsize_y = traj[1][i].coeffs.size();
			seg_msg.coeffsize_z = traj[1][i].coeffs.size();
			seg_msg.coeffsize_yaw = traj[1][i].coeffs.size();

			for(auto& coeff:traj[0][i].coeffs)
				seg_msg.coeffs_x.push_back(coeff);
			for(auto& coeff:traj[1][i].coeffs)
				seg_msg.coeffs_y.push_back(coeff);
			for(auto& coeff:traj[1][i].coeffs)
				seg_msg.coeffs_z.push_back(0);
			for(auto& coeff:traj[1][i].coeffs)
				seg_msg.coeffs_yaw.push_back(0);

			seg_msg.t_start = 0;
			seg_msg.t_end = 1.0;

			poly_msg.segments.push_back(seg_msg);
		}

		lcm->publish("quad_planner/polynomial_curve", &poly_msg);

		std::cout << "traj sent to lcm" << std::endl;

	} catch(GRBException e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} catch(...) {
		std::cout << "Exception during optimization" << std::endl;
	}

	return 0;
}


