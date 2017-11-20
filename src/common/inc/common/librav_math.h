/* 
 * librav_math.h
 * 
 * Created on: Nov 06, 2017 14:38
 * Description:   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef LIBRAV_MATH_H
#define LIBRAV_MATH_H

#include <cstdint>
#include <cmath>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "common/librav_types.h"

namespace librav
{

namespace PolynomialMath
{
    using PolynomialCoeffs = Eigen::Array<float,1, Eigen::Dynamic>;

    void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs);
    double GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau);
}

namespace TransformationMath 
{
    
    typedef struct {
        Eigen::Translation<double,3> trans;
        Eigen::Quaterniond quat;
    } Transform3D;
    
    typedef Eigen::Translation<double,3> Translation3D;
    
    Position3Dd TransformPosition3D(Transform3D transform, Position3Dd pos);
    
}

}

#endif /* LIBRAV_MATH_H */
