/* 
 * polyopt_math.cpp
 * 
 * Created on: Aug 23, 2016
 * Description: 
 * Reference:
 *  [1] https://github.com/MUNEEBABBASI/2DQuadSim
 *  [2] Mellinger, D., and V. Kumar. 2011. “Minimum Snap Trajectory Generation and Control for Quadrotors.”
 *  	In Robotics and Automation (ICRA), 2011 IEEE International Conference on, 2520–25.
 *  [3] Richter, Charles, Adam Bry, and Nicholas Roy. 2013/5. “Polynomial Trajectory Planning for Quadrotor Flight.”
 *  	In. http://rss2013_uav.visual-navigation.com/pdf/richter_rss13_workshop.pdf.
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <iostream>
#include <cmath>
#include <limits>

#include "polyopt/polyopt_math.h"

using namespace librav;
using namespace Eigen;

void PolyOptMath::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	int64_t N = poly_order;
	int64_t r = deriv_order;

	coeffs = ArrayXXf::Ones(1, N+1);

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

uint32_t PolyOptMath::GetNonZeroCoeffNum(const Eigen::Ref<const PolynomialCoeffs> coeffs)
{
	uint32_t order = 0;

	PolynomialCoeffs data = coeffs;
	for(int32_t i = 0; i < data.cols(); i++)
		if(data(i) != 0) order++;

	return order;
}

double PolyOptMath::GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau)
{
	double val = 0;
	int64_t N = coeffs.size() - 1;
	int64_t r = deriv_order;

	PolynomialCoeffs deriv_coeff(N + 1);
	GetDerivativeCoeffs(N, r, deriv_coeff);

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

void PolyOptMath::GetDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1,
		Eigen::Ref<Eigen::MatrixXf> q)
{
	int64_t N = poly_order;
	int64_t r = deriv_order;

	q = MatrixXf::Zero(N+1, N+1);

	for(int64_t i = 0; i <= N; i++)
		for(int64_t j = 0; j <= N; j++)
		{
			if(i >= r && j >= r)
			{
				uint64_t multiply = 1;
				uint64_t order = i + j - 2 * r + 1;

				for(int64_t m = 0; m <= r - 1; m++)
				{
					multiply *= (i - m) * (j - m);
				}

				//q(i,j) = 2.0 * multiply * (std::pow(t1, order) - std::pow(t0, order))/order;
				q(i,j) = multiply * (std::pow(t1, order) - std::pow(t0, order))/order;
			}
		}
}

void PolyOptMath::GetNonDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1,
		Eigen::Ref<Eigen::MatrixXf> q)
{
	GetDimQMatrix(poly_order, deriv_order, 0.0, 1.0, q);

	double nondim_coeff;
	nondim_coeff = 1.0/(std::pow((t1 - t0), 2*deriv_order - 1));

	q = nondim_coeff * q;

	// flip Q so that the coefficients are arranged from sigma_n to sigma_0
	Eigen::MatrixXf q_flip = q;
	uint64_t row_size = q_flip.cols();
	uint64_t col_size = q_flip.rows();
	q_flip = Eigen::MatrixXf::Zero(row_size, col_size);

	for(int64_t i = 0; i < row_size; i++)
		for(int64_t j = 0; j < col_size; j++)
			q_flip(i,j) = q(row_size - 1 - i, col_size - 1 - j);

	q = q_flip;
}

void PolyOptMath::GetNonDimQMatrices(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, Eigen::Ref<Eigen::MatrixXf> q)
{
	int64_t N = poly_order;
	int64_t r = deriv_order;
	int64_t seg_num = keyframe_num - 1;

	q = MatrixXf::Zero(seg_num*(N+1), seg_num*(N+1));

	for(uint64_t i = 0; i < seg_num; i++)
	{
		Eigen::MatrixXf sub_q = MatrixXf::Zero(N+1, N+1);
		GetNonDimQMatrix(N, r, keyframe_ts(0, i), keyframe_ts(0, i+1), sub_q);

		for(uint64_t row = 0; row < sub_q.rows(); row++)
			for(uint64_t col = 0; col < sub_q.cols(); col++)
				q(i*(N+1) + row, i*(N+1) + col) = sub_q(row,col);
	}
}

void PolyOptMath::GetNonDimEqualityConstrs(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_eq, Eigen::Ref<Eigen::MatrixXf> b_eq)
{
	// TODO check if size of keyframe values and that of keyframe time stamps match
	// TODO check if keyframe_num is greater than 1

	int64_t N = poly_order;
	int64_t r = deriv_order;
	int64_t traj_seg_num = keyframe_num - 1;

	A_eq = MatrixXf::Zero(traj_seg_num * 2 * r, traj_seg_num * (N + 1));
	b_eq = MatrixXf::Zero(traj_seg_num * 2 * r, 1);

	// check each piece of trajectory
	for(int64_t j = 0; j < traj_seg_num; j++)
	{
		// check each derivative
		for(int64_t i = 0; i <= r-1; i++)
		{
			PolynomialCoeffs coeff(N + 1);
			GetDerivativeCoeffs(poly_order, i, coeff);

			// if there is only 1 segment, no continuity needs to be considered
			if(traj_seg_num == 1)
			{
				double nondim_coeff = 1/(std::pow(keyframe_ts(0, j+1)-keyframe_ts(0, j),i));
				for(int64_t k = 0; k < N + 1; k++)
				{
					// A(tau_0), A(tau_1)
					if(N - k >= i)
					{
						A_eq(j*2*r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(0, N - k - i );
						A_eq(j*2*r + r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(1, N - k - i );
					}
					else
					{
						A_eq(j*2*r + i, j*(N + 1) + k) = 0;
						A_eq(j*2*r + r + i, j*(N + 1) + k) = 0;
					}
				}

				b_eq(j*2*r + i,0) = keyframe_vals(i, j);
				b_eq(j*2*r + r + i,0) = keyframe_vals(i, j + 1);
			}
			// if there are more segments, intermediate key frames need to be continuous
			else
			{
				double nondim_coeff = 1/(std::pow(keyframe_ts(0, j+1)-keyframe_ts(0, j),i));

				// special case: if no constraint is specified, just ensure continuity
				if(keyframe_vals(i, j) == std::numeric_limits<float>::infinity())
				{
					double nondim_coeff_prev = 1/(std::pow(keyframe_ts(0, j)-keyframe_ts(0, j-1),i));

					for(int64_t k = 0; k < N + 1; k++)
					{
						// A(tau_0), A(tau_1)
						if(N - k >= i)
						{
							A_eq(j*2*r + i, (j-1)*(N + 1) + k) = nondim_coeff_prev * coeff(k)*std::pow(1, N - k);
							A_eq(j*2*r + i, j*(N + 1) + k) = - nondim_coeff * coeff(k)*std::pow(0, N - k - i);
						}
						else
							A_eq(j*2*r + i, j*(N + 1) + k) = 0;
					}

					b_eq(j*2*r + i,0) = 0;
				}
				else
				{
					for(int64_t k = 0; k < N + 1; k++)
					{
						// A(tau_0), A(tau_1)
						if(N - k >= i)
						{
							A_eq(j*2*r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(0, N - k - i);
						}
						else
						{
							A_eq(j*2*r + i, j*(N + 1) + k) = 0;
						}
					}

					b_eq(j*2*r + i,0) = keyframe_vals(i, j);
				}

				if(keyframe_vals(i, j+1) == std::numeric_limits<float>::infinity())
				{
					for(int64_t k = 0; k < N + 1; k++)
					{
						// A(tau_0), A(tau_1)
						A_eq(j*2*r + r + i, j*(N + 1) + k) = 0;
					}
					b_eq(j*2*r + r + i,0) = 0;
				}
				else
				{
					for(int64_t k = 0; k < N + 1; k++)
					{
						// A(tau_0), A(tau_1)
						if(N - k >= i)
						{
							A_eq(j*2*r + r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(1, N - k - i);
						}
						else
						{
							A_eq(j*2*r + r + i, j*(N + 1) + k) = 0;
						}
					}

					b_eq(j*2*r + r + i,0) = keyframe_vals(i, j+1);
				}
			}
		}
	}

//	std::cout << "A_eq:\n" << A_eq << std::endl;
//	std::cout << "b_eq:\n" << b_eq << std::endl;
}

void PolyOptMath::GetNonDimCorridorConstrs(uint32_t poly_order, uint32_t keyframe_num, uint32_t midpoint_num, double max_dist,
		const std::vector<Eigen::MatrixXf>& keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_cor, Eigen::Ref<Eigen::MatrixXf> b_cor)
{
	uint8_t dim = keyframe_vals.size();
	int64_t N = poly_order;
	int64_t nc = midpoint_num;
	int64_t traj_seg_num = keyframe_num - 1;

	A_cor = MatrixXf::Zero(nc * 2 * dim * traj_seg_num, (N + 1) * dim * traj_seg_num);
	b_cor = MatrixXf::Zero(nc * 2 * dim * traj_seg_num, 1);

	// calculate distance between each two keyframes
	std::vector<double> dist;
	dist.reserve(keyframe_num - 1);

	for(int i = 0; i < traj_seg_num; i++)
	{
		std::vector<double> dist_error;
		double sum = 0;

		for(int j = 0; j < dim; j++)
			dist_error.push_back(keyframe_vals[j](0, i) - keyframe_vals[j](0, i + 1));

		for(auto& error:dist_error)
			sum += std::pow(error,2);

		dist.push_back(std::sqrt(sum));
	}

	// check each segment
	for(int i = 0; i < traj_seg_num; i++)
	{
		// check each dimension
		for(int p = 0; p < dim; p++)
		{
			// calculate const
			double const_sum = 0;
			double const_dim = 0;
			for(int j = 0; j < dim; j++)
				const_sum += keyframe_vals[j](0, i) * (keyframe_vals[j](0, i + 1) - keyframe_vals[j](0, i));

			const_dim = keyframe_vals[p](0, i) - const_sum * (keyframe_vals[p](0, i + 1) - keyframe_vals[p](0, i))/std::pow(dist[i],2);

			// calculate coefficients for each dimension
			double coeff_sum = 0;
			MatrixXf const_coeff = MatrixXf::Zero(dim, 1);
			for(int j = 0; j < dim; j++)
			{
				if(j == p)
					const_coeff(j,0) = 1 - std::pow(keyframe_vals[j](0, i + 1) - keyframe_vals[j](0, i), 2)/std::pow(dist[i],2);
				else
					const_coeff(j,0) = - (keyframe_vals[j](0, i + 1) - keyframe_vals[j](0, i))
						*(keyframe_vals[p](0, i + 1) - keyframe_vals[p](0, i))/std::pow(dist[i], 2);
			}

			// add nc middle points
			for(int n = 0; n < nc ; n++)
			{
				double tau = (n + 1)/(1.0 + nc);

				MatrixXf coeffs = MatrixXf::Zero(dim, N + 1);

				for(int k = 0; k < dim; k++) {
					for(int l = 0; l <= N; l++) {
						// round values to 0.0001
						coeffs(k, l) = std::round(std::pow(tau, N - l) * const_coeff(k, 0) * 10000.0)/10000.0;
					}

					// A_cor = MatrixXf::Zero(nc * 2 * dim * traj_seg_num, (N + 1) * dim * traj_seg_num);
					for(int idx = 0; idx < N + 1; idx++)
					{
						A_cor(nc*2*dim*i + nc*2*p + 2*n, (N+1)*traj_seg_num*k + (N+1)*i + idx) = coeffs(k, idx);
						A_cor(nc*2*dim*i + nc*2*p + 2*n + 1, (N+1)*traj_seg_num*k + (N+1)*i + idx) = -coeffs(k, idx);
					}
				}

				b_cor(nc*2*dim*i + nc*2*p + 2*n, 0) = max_dist + const_dim;
				b_cor(nc*2*dim*i + nc*2*p + 2*n + 1, 0) = max_dist - const_dim;
			}
		}
	}

//	std::cout << "A_cor: \n" << A_cor << std::endl;
//	std::cout << "size A_cor: " << nc * 2 * dim * traj_seg_num << " , " << (N + 1) * dim * traj_seg_num << std::endl;
//	std::cout << "b_cor: \n" << b_cor << std::endl;
//	std::cout << "size b_cor: " << nc * 2 * dim * traj_seg_num << std::endl;
}
