/* 
 * quad_mixer.hpp
 * 
 * Created on: Jun 25, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef QUAD_MIXER_HPP
#define QUAD_MIXER_HPP

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "control/quad_state.hpp"

namespace librav {

class QuadMixer
{
public:
	QuadMixer(const QuadState& _rs);

	QuadCmd CalcMotorCmd(float force, float toqure[3], QuadFlightType type);
	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);
	
private:
	const QuadState& state_;

	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;	
};

}

#endif /* QUAD_MIXER_HPP */
