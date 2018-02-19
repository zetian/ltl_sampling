/* 
 * gurobi_utils.h
 * 
 * Created on: Aug 28, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef GUROBI_UTILS_H
#define GUROBI_UTILS_H

#include <cstdint>
#include <vector>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

namespace librav {

namespace GurobiUtils {

void GetQuadraticCostFuncExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> Q, uint32_t x_size, GRBQuadExpr& expr);
void AddLinEqualityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_eq, const Eigen::Ref<const Eigen::MatrixXf> b_eq,
				uint32_t x_size, uint32_t constr_size, GRBModel& model);
void AddLinInequalityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_ineq, const Eigen::Ref<const Eigen::MatrixXf> b_ineq,
				uint32_t x_size, uint32_t constr_size, GRBModel& model);
}

}

#endif /* GUROBI_UTILS_H */
