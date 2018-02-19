/*
 * test_qp.cpp
 *
 *  Created on: Aug 25, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cstdint>
#include <limits>
#include <cstring>
#include <vector>

#include "gurobi_c++.h"

#include "eigen3/Eigen/Core"

// opencv
#include "opencv2/opencv.hpp"

#include "polyopt/polyopt_math.h"
#include "polyopt/gurobi_utils.h"

using namespace std;
using namespace librav;
using namespace Eigen;
using namespace cv;

int main(int   argc, char *argv[])
{
	uint32_t r = 2;
	uint32_t N = 2 * r - 1;
	MatrixXf Q = MatrixXf::Zero(N+1, N+1);
	MatrixXf A_eq = MatrixXf::Zero(2 * r, 1 * (N + 1));
	MatrixXf b_eq = MatrixXf::Zero(2 * r, 1);

	MatrixXf keyframe_vals = MatrixXf::Zero(r, 2);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, 2);

	keyframe_vals(0,0) = -0.15;
	keyframe_vals(0,1) = 0.25;
	keyframe_vals(1,0) = 0.1;
	keyframe_vals(1,1) = 0.1;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;

	PolyOptMath::GetNonDimQMatrix(N,r,0,1.2,Q);
	PolyOptMath::GetNonDimEqualityConstrs(N, r, 2, keyframe_vals, keyframe_ts, A_eq, b_eq);

	std::cout << "\nQ: \n" << Q << std::endl;
	std::cout << "\nA_eq:\n" << A_eq << std::endl;
	std::cout << "\nb_eq:\n" << b_eq << std::endl;

	try {
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		// Create variables
//		GRBVar sig0 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig0");
//		GRBVar sig1 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig1");
//		GRBVar sig2 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig2");
//		GRBVar sig3 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig3");

		std::vector<GRBVar> sig;
		sig.resize(4);
		for(int i = 0; i < 4; i++)
		{
			std::string var_name = "sig"+std::to_string(i);
			sig[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}

		// Integrate new variables
		model.update();

		// Set objective
//		GRBQuadExpr obj = sig[0]*((125.0*sig[0])/18.0 + (125.0*sig[1])/36.0) + sig[1]*((125.0*sig[0])/36.0 + (125.0*sig[1])/54.0);
//		model.setObjective(obj);

		GRBQuadExpr cost_fun;
//		std::vector<GRBQuadExpr> temp_expr;
//		temp_expr.resize(4);
//		for(int i = 0; i < 4; i++)
//		{
//			for(int j = 0; j < 4; j++)
//				if(Q(j,i)!=0)
//					temp_expr[i] += sig[j] * Q(j,i);
//			std::cout << "idx: " << i << " "<< temp_expr[i] << std::endl;
//		}
//		for(int i = 0; i < 4; i++)
//			cost_fun += temp_expr[i].getLinExpr() * sig[i];
//		std::cout << cost_fun << std::endl;
		GurobiUtils::GetQuadraticCostFuncExpr(sig, Q, 4, cost_fun);;
		model.setObjective(cost_fun);

		// Add constraints
//		model.addConstr(sig[3] == -0.15, "c0");
//		model.addConstr((5.0*sig[2])/6.0 == 0, "c1");
//		model.addConstr(sig[0] + sig[1] + sig[2] + sig[3] == 0.25, "c2");
//		model.addConstr((5.0*sig[0])/2.0 + (5.0*sig[1])/3.0 + (5.0*sig[2])/6.0 == 0, "c3");

//		for(int i = 0; i < 4; i++)
//		{
//			GRBLinExpr constr;
//			for(int j = 0; j < 4; j++)
//				constr += sig[j] * A_eq(i,j);
//
//			//std::cout << "constraint " << i << " : " << constr << " = " << b_eq(i,0) << std::endl;
//			std::string constr_name = "c"+std::to_string(i);
//			model.addConstr(constr == b_eq(i, 0), constr_name);
//		}
		GurobiUtils::AddLinEqualityConstrExpr(sig, A_eq, b_eq, 4, 4, model);

		// Optimize model
		model.optimize();

		std::cout << sig[0].get(GRB_StringAttr_VarName) << " "
				<< sig[0].get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig[1].get(GRB_StringAttr_VarName) << " "
				<< sig[1].get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig[2].get(GRB_StringAttr_VarName) << " "
				<< sig[2].get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig[3].get(GRB_StringAttr_VarName) << " "
				<< sig[3].get(GRB_DoubleAttr_X) << std::endl;

		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

		std::vector<double> result_coeffs;
		for(auto& s:sig)
		{
			result_coeffs.push_back(s.get(GRB_DoubleAttr_X));
		}
//		cv::Mat vis_img;
//		CurveVis::VisPolynomialCurve(result_coeffs, 0, 1.2, 0.1, vis_img);
//
//		// display visualization result
//		namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
//		imshow("Processed Image", vis_img);
//
//		waitKey(0);
		std::cout << PolyOptMath::GetPolynomialValue(result_coeffs, 0, 0) << std::endl;
		std::cout << PolyOptMath::GetPolynomialValue(result_coeffs, 0, 1.0) << std::endl;
		std::cout << PolyOptMath::GetPolynomialValue(result_coeffs, 1, 0) << std::endl;
		std::cout << PolyOptMath::GetPolynomialValue(result_coeffs, 1, 1.0) << std::endl;

	} catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	} catch(...) {
		cout << "Exception during optimization" << endl;
	}

	return 0;
}


