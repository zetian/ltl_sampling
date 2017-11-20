/* 
 * polyopt_math.h
 * 
 * Created on: Aug 23, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef POLYOPT_MATH_H
#define POLYOPT_MATH_H

#include <vector>
#include <cstdint>

#include "eigen3/Eigen/Core"

namespace librav {

using PolynomialCoeffs = Eigen::Array<float,1, Eigen::Dynamic>;

namespace PolyOptMath {

/****** Minimum-snap Trajectory Optimization ******/
// helper functions
void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs);
uint32_t GetNonZeroCoeffNum(const Eigen::Ref<const PolynomialCoeffs> coeffs);
double GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau);

// Q matrix
void GetDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1, Eigen::Ref<Eigen::MatrixXf> q);
void GetNonDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1, Eigen::Ref<Eigen::MatrixXf> q);
void GetNonDimQMatrices(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, Eigen::Ref<Eigen::MatrixXf> q);

// Equality constraints
void GetNonDimEqualityConstrs(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_eq, Eigen::Ref<Eigen::MatrixXf> b_eq);

// Corridor constraints
void GetNonDimCorridorConstrs(uint32_t poly_order, uint32_t keyframe_num, uint32_t midpoint_num, double max_dist,
		const std::vector<Eigen::MatrixXf>& keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_cor, Eigen::Ref<Eigen::MatrixXf> b_cor);
}
}

#endif /* POLYOPT_MATH_H */
