/* 
 * gurobi_polyopt.cpp
 * 
 * Created on: Aug 29, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <limits>
#include <ctime>
#include <string>

#include "polyopt/polyopt_math.h"
#include "polyopt/gurobi_solver/gurobi_polyopt.h"
#include "polyopt/gurobi_solver/gurobi_utils.h"

using namespace librav;
using namespace Eigen;

GRBEnv GurobiPolyOpt::grb_env_ = GRBEnv();

GurobiPolyOpt::GurobiPolyOpt():
		r_(4), N_(10),
		kf_num_(2)
{
}

GurobiPolyOpt::~GurobiPolyOpt()
{

}

void GurobiPolyOpt::InitCalcVars()
{
	Q_ = MatrixXf::Zero((kf_num_ - 1) * (N_+1), (kf_num_ - 1) * (N_+1));
	A_eq_ = MatrixXf::Zero((kf_num_ - 1) * 2 * r_, (kf_num_ - 1) * (N_ + 1));
	b_eq_ = MatrixXf::Zero((kf_num_ - 1) * 2 * r_, 1);

	MatrixXf keyframe_vals = MatrixXf::Zero(r_, kf_num_);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num_);
}

OptResultCurve GurobiPolyOpt::OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order)
{
	OptResultCurve result;
	result.cost = -1;

	kf_num_ = keyframe_num;
	r_ = deriv_order;
	N_ = poly_order;

	if(N_ < 2*r_ - 1)
	{
		N_ = 2*r_ - 1;
		std::cerr << "Inappropriate order is specified for the polynomial to optimize: N < 2*r - 1" << std::endl;
	}

	InitCalcVars();

	try {
		// record start time
		clock_t exec_time;
		exec_time = clock();

		// create a optimization model
		GRBModel model = GRBModel(grb_env_);

		// add variables to model
		std::vector<GRBVar> x;
		uint32_t var_num = (N_ + 1)*(kf_num_ - 1);
		x.resize(var_num);
		for(int i = 0; i < var_num; i++)
		{
			std::string var_name = "sigma"+std::to_string(i);
			x[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}
		model.update();

		// set objective
		GRBQuadExpr cost_fun;
		PolyOptMath::GetNonDimQMatrices(N_,r_,kf_num_, keyframe_ts, Q_);
		GurobiUtils::GetQuadraticCostFuncExpr(x, Q_, var_num, cost_fun);;
		model.setObjective(cost_fun);

		// add constraints
		PolyOptMath::GetNonDimEqualityConstrs(N_, r_, kf_num_, keyframe_vals, keyframe_ts, A_eq_, b_eq_);
		GurobiUtils::AddLinEqualityConstrExpr(x, A_eq_, b_eq_, var_num, var_num, model);

		// optimize model
		model.optimize();

		exec_time = clock() - exec_time;
		std::cout << "Optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

		for(int i = 0; i < var_num; i++)
		{
			if(i!=0 && i%(N_+1) == 0) {
				std::cout << std::endl;
			}

			std::cout << x[i].get(GRB_StringAttr_VarName) << " : "
					<< x[i].get(GRB_DoubleAttr_X) << std::endl;
		}

		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

		// pack optimization result data
		std::vector<double> seg_coeffs;
		std::vector<std::vector<double>> all_coeffs;
		all_coeffs.resize(kf_num_ - 1);
		for(int i = 0; i < var_num; i++)
		{
			all_coeffs[i/(N_+1)].push_back(x[i].get(GRB_DoubleAttr_X));
		}

		uint64_t idx = 0;
		for(auto& seg:all_coeffs)
		{
			CurveParameter param;
			param.coeffs = all_coeffs[idx];
			param.ts = keyframe_ts(0, idx);
			param.te = keyframe_ts(0, idx+1);

			result.segments.push_back(param);
			idx++;
		}
		result.cost = model.get(GRB_DoubleAttr_ObjVal);

		return result;

	} catch(GRBException& e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} catch(...) {
		std::cout << "Exception during optimization" << std::endl;
	}

	return result;
}

OptResultParam GurobiPolyOpt::OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
				const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
				const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
				uint32_t keyframe_num, uint32_t var_size)
{
	OptResultParam result;
	result.cost = -1;

	kf_num_ = keyframe_num;
	InitCalcVars();

	try {
		// record start time
		clock_t exec_time;
		exec_time = clock();

		// create a optimization model
		GRBModel model = GRBModel(grb_env_);

		// add variables to model
		std::vector<GRBVar> x;
		x.resize(var_size);
		for(int i = 0; i < var_size; i++)
		{
			std::string var_name = "sigma"+std::to_string(i);
			x[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}
		model.update();

		// set objective
		GRBQuadExpr cost_fun;
		GurobiUtils::GetQuadraticCostFuncExpr(x, Q_m, var_size, cost_fun);;
		model.setObjective(cost_fun);

		// add constraints
		GurobiUtils::AddLinEqualityConstrExpr(x, Aeq_m, beq_m, var_size, var_size, model);

		// optimize model
		model.optimize();

		exec_time = clock() - exec_time;
		std::cout << "Optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

		// display and pack optimization result data
		for(int i = 0; i < var_size; i++)
		{
//			if(i!=0 && i%(var_size/(keyframe_num-1)) == 0) {
//				std::cout << std::endl;
//			}
			result.params.push_back(x[i].get(GRB_DoubleAttr_X));
//			std::cout << x[i].get(GRB_StringAttr_VarName) << " : "
//					<< x[i].get(GRB_DoubleAttr_X) << std::endl;
		}
		result.cost = model.get(GRB_DoubleAttr_ObjVal);
		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

		return result;

	} catch(GRBException& e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} catch(...) {
		std::cout << "Exception during optimization" << std::endl;
	}

	return result;
}

OptResultParam GurobiPolyOpt::OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
		const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
		const Eigen::Ref<const Eigen::MatrixXf> Aineq_m, const Eigen::Ref<const Eigen::MatrixXf> bineq_m,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		uint32_t keyframe_num, uint32_t var_size)
{
	OptResultParam result;
	result.cost = -1;

	kf_num_ = keyframe_num;
	InitCalcVars();

	try {
		// record start time
		clock_t exec_time;
		exec_time = clock();

		// create a optimization model
		GRBModel model = GRBModel(grb_env_);

		// add variables to model
		std::vector<GRBVar> x;
		x.resize(var_size);
		for(int i = 0; i < var_size; i++)
		{
			std::string var_name = "sigma"+std::to_string(i);
			x[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}
		model.update();

		// set objective
		GRBQuadExpr cost_fun;
		GurobiUtils::GetQuadraticCostFuncExpr(x, Q_m, var_size, cost_fun);;
		model.setObjective(cost_fun);

		// add constraints
		GurobiUtils::AddLinEqualityConstrExpr(x, Aeq_m, beq_m, var_size, beq_m.rows(), model);
		GurobiUtils::AddLinInequalityConstrExpr(x, Aineq_m, bineq_m, var_size, bineq_m.rows(), model);

		// optimize model
		model.optimize();

		exec_time = clock() - exec_time;
		std::cout << "Optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

		// display and pack optimization result data
		for(int i = 0; i < var_size; i++)
		{
			//			if(i!=0 && i%(var_size/(keyframe_num-1)) == 0) {
			//				std::cout << std::endl;
			//			}
			result.params.push_back(x[i].get(GRB_DoubleAttr_X));
			//			std::cout << x[i].get(GRB_StringAttr_VarName) << " : "
			//					<< x[i].get(GRB_DoubleAttr_X) << std::endl;
		}
		result.cost = model.get(GRB_DoubleAttr_ObjVal);
		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

		return result;

	} catch(GRBException& e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} catch(...) {
		std::cout << "Exception during optimization" << std::endl;
	}

	return result;
}
