/*
 * test_utils.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cstdint>
#include <limits>

#include "gurobi_c++.h"

#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_math.h"

using namespace std;
using namespace librav;
using namespace Eigen;

int main(int   argc, char *argv[])
{
	/* test coefficients */
	PolynomialCoeffs coeff(8);
//	po_utils.GetDerivativeCoeffs(7, 0, coeff);
//	std::cout << coeff << std::endl;
//	po_utils.GetDerivativeCoeffs(7, 1, coeff);
//	std::cout << coeff << std::endl;
//	po_utils.GetDerivativeCoeffs(7, 2, coeff);
//	std::cout << coeff << std::endl;
//	po_utils.GetDerivativeCoeffs(7, 3, coeff);
//	std::cout << coeff << std::endl;
//	po_utils.GetDerivativeCoeffs(7, 4, coeff);
//	std::cout << coeff << std::endl;
//	std::cout << po_utils.GetNonZeroCoeffNum(coeff) << std::endl;

	uint32_t r = 2;
	uint32_t N = 2 * r - 1;

	/* test Q matrix */
//	MatrixXf q = MatrixXf::Zero(N+1, N+1);
	//po_utils.GetDimQMatrix(3,2,0,1.2,q);
//	PolyOptUtils::GetNonDimQMatrix(N,r,0,1.2,q);
//	std::cout << "matrix Q: " << std::endl;
//	std::cout << q << std::endl;

	/* test infinity */
//	float inf1 = std::numeric_limits<float>::infinity();
//	float inf2 = std::numeric_limits<float>::infinity();
//	if(inf1 == inf2)
//		std::cout << "infinity can be compared" << std::endl;

	/* test equality constraints */
//	MatrixXf keyframe_vals = MatrixXf::Zero(r, 2);
//	MatrixXf keyframe_ts = MatrixXf::Zero(1, 2);
//
//	keyframe_vals(0,0) = -0.15;
//	keyframe_vals(0,1) = 0.25;
//	keyframe_vals(1,0) = 0.1;
//	keyframe_vals(1,1) = 0.2;
//
//	keyframe_ts(0,0) = 0;
//	keyframe_ts(0,1) = 1.2;
//
//	MatrixXf A_eq = MatrixXf::Zero(2 * r, 1 * (N + 1));
//	MatrixXf b_eq = MatrixXf::Zero(2 * r, 1);
//
//	PolyOptUtils::GetNonDimEqualityConstrs(N, r, 2, keyframe_vals, keyframe_ts, A_eq, b_eq);

	//std::cout << "test: " << std::pow(0, 1) << std::endl;

	/* test multiple key frames */
	uint8_t kf_num = 3;
	MatrixXf q = MatrixXf::Zero(2*(N+1), 2*(N+1));

	MatrixXf keyframe_vals = MatrixXf::Zero(r, 3);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, 3);

	keyframe_vals(0,0) = -0.15;
	keyframe_vals(0,1) = 0.25;
	keyframe_vals(0,2) = 0.3;

	keyframe_vals(1,0) = 0;
	keyframe_vals(1,1) = std::numeric_limits<float>::infinity();
	keyframe_vals(1,2) = 0;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;
	keyframe_ts(0,2) = 3;

//	PolyOptUtils::GetNonDimQMatrices(N,r,kf_num, keyframe_ts,q);
//	std::cout << "matrix Q: " << std::endl;
//	std::cout << q << std::endl;

	MatrixXf A_eq = MatrixXf::Zero((kf_num - 1) * 2 * r, (kf_num - 1) * (N + 1));
	MatrixXf b_eq = MatrixXf::Zero((kf_num - 1) * 2 * r, 1);

	PolyOptMath::GetNonDimEqualityConstrs(N, r, kf_num, keyframe_vals, keyframe_ts, A_eq, b_eq);
}

