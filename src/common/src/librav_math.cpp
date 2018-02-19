/* 
 * librav_math.cpp
 * 
 * Created on: Nov 06, 2017 14:41
 * Description:   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "common/librav_math.hpp"

using namespace librav;

void PolynomialMath::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	{
		int64_t N = poly_order;
		int64_t r = deriv_order;

		coeffs = Eigen::ArrayXXf::Ones(1, N+1);

		/* if deriv_order == 0, no derivative is taken */
		if(r == 0)
			return;

		/* otherwise calculate derivative coefficients */
		int64_t n;
		for(n = 0; n <= N; n++)
		{
			if(n < r)
				coeffs(0, N - n) = 0;
			else
			{
				uint64_t multiply = 1;
				for(uint64_t m = 0; m <= r - 1; m++)
				{
					multiply *= (n - m);
				}
				coeffs(0,N - n) = multiply;
			}
		}
	}
}

double PolynomialMath::GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau)
{
	double val = 0;
	int64_t N = coeffs.size() - 1;
	int64_t r = deriv_order;

	PolynomialCoeffs deriv_coeff(N + 1);
	PolynomialMath::GetDerivativeCoeffs(N, r, deriv_coeff);

	uint32_t coeff_size = coeffs.size();
	for(int i = 0; i <= N; i++)
	{
		double item_val;

		if(N - i >= r) {
			item_val =  deriv_coeff[i] * coeffs[i] * std::pow(tau, N - i - r);

			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}


/**
 * frames: base_frame, frame1
 * pos: the interested position defined in frame1
 * transform: the orientation of frame 1 defined in base_frame
 * return value: the interested position defined in base_frame
 */
Position3Dd TransformationMath::TransformPosition3D(Transform3D transform, Position3Dd pos)
{
	Eigen::Vector3d output = transform.trans * transform.quat * Eigen::Vector3d(pos.x, pos.y, pos.z);

	return Position3Dd(output[0], output[1], output[2]);
}
