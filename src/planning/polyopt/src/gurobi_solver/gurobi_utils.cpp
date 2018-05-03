/* 
 * gurobi_utils.cpp
 * 
 * Created on: Aug 28, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "polyopt/gurobi_solver/gurobi_utils.h"

using namespace librav;

void GurobiUtils::GetQuadraticCostFuncExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> Q, uint32_t x_size, GRBQuadExpr& expr)
{
	// TODO check if size matches

	std::vector<GRBQuadExpr> temp_expr;
	temp_expr.resize(x_size);

	for(int i = 0; i < x_size; i++)
	{
		for(int j = 0; j < x_size; j++)
			if(Q(j,i)!=0)
				temp_expr[i] += x[j] * Q(j,i);
		//std::cout << "idx: " << i << " "<< temp_expr[i] << std::endl;
	}

	for(int i = 0; i < x_size; i++)
		expr += temp_expr[i].getLinExpr() * x[i];

//	std::cout << "\nCost function: " << std::endl;
//	std::cout << expr << std::endl;
//	std::cout << "\n--------------------------------------------\n" << std::endl;
}

void GurobiUtils::AddLinEqualityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_eq, const Eigen::Ref<const Eigen::MatrixXf> b_eq,
				uint32_t x_size, uint32_t constr_size, GRBModel& model)
{
	std::cout << "equality constraint size: " << constr_size << std::endl;
	for(int i = 0; i < constr_size; i++)
	{
		GRBLinExpr constr;
		for(int j = 0; j < x_size; j++)
			constr += x[j] * A_eq(i,j);

		//std::cout << "constraint " << i << " : " << constr << " = " << b_eq(i,0) << std::endl;
		std::string constr_name = "ec"+std::to_string(i);
		model.addConstr(constr == b_eq(i, 0), constr_name);
	}
}

void GurobiUtils::AddLinInequalityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_ineq, const Eigen::Ref<const Eigen::MatrixXf> b_ineq,
		uint32_t x_size,uint32_t constr_size, GRBModel& model)
{
	for(int i = 0; i < constr_size; i++)
	{
		GRBLinExpr constr;
		for(int j = 0; j < x_size; j++)
			constr += x[j] * A_ineq(i,j);

		//std::cout << "constraint " << i << " : " << constr << " = " << b_eq(i,0) << std::endl;
		std::string constr_name = "inec"+std::to_string(i);
		model.addConstr(constr <= b_ineq(i, 0), constr_name);
	}
}

