/* 
 * gurobi_polyopt.h
 * 
 * Created on: Aug 29, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef GUROBI_POLYOPT_H
#define GUROBI_POLYOPT_H

#include "common/poly_curve.h"

#include <cstdint>
#include <iostream>
#include <string>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

namespace librav {

typedef struct {
	std::vector<CurveParameter> segments;
	double cost;

	void print() {
		std::cout << "\n**************************************\n" << std::endl;
		std::cout << "cost: " << cost << std::endl;

		uint32_t idx = 0;
		for(auto& seg:segments)
		{
			std::cout << "\nseg " << idx << " : " << std::endl;
			uint32_t coeff_idx = 0;
			for(auto& coef:seg.coeffs)
				std::cout << coeff_idx++ << " : " << coef << std::endl;
			std::cout << "start time: " << seg.ts << " , end time: " << seg.te << std::endl;
			idx++;
		}
		std::cout << "\n**************************************\n" << std::endl;
	}
} OptResultCurve;

typedef struct {
	std::vector<double> params;
	double cost;
} OptResultParam;

class GurobiPolyOpt {
public:
	GurobiPolyOpt();
	~GurobiPolyOpt();

private:
	// only one environment needed
	static GRBEnv grb_env_;

	// derivative to optimize
	uint32_t r_;
	// highest order of polynomial
	uint32_t N_;
	// number of key frames
	uint32_t kf_num_;

	Eigen::MatrixXf Q_;
	Eigen::MatrixXf A_eq_;
	Eigen::MatrixXf b_eq_;

private:
	void InitCalcVars();

public:
	// optimize a one-dimensional trajectory, returns a curve data structure
	OptResultCurve OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
				const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order);

	// jointly optimize a multi-dimensional trajectory, returns optimized parameters
	OptResultParam OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
				const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
				const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
				uint32_t keyframe_num, uint32_t var_size);
	OptResultParam OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
			const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
			const Eigen::Ref<const Eigen::MatrixXf> Aineq_m, const Eigen::Ref<const Eigen::MatrixXf> bineq_m,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
			uint32_t keyframe_num, uint32_t var_size);
};

}

#endif /* GUROBI_POLYOPT_H */
