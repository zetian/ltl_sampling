/* 
 * quad_mixer.cpp
 * 
 * Created on: Jun 25, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include "control/quad_mixer.hpp"

using namespace librav;

QuadMixer::QuadMixer(const QuadState& _rs):
    state_(_rs)
{
    double d = state_.arm_length_;
	double c = state_.kM_/state_.kF_;

	plus_type_trans_ << 1, 1, 1, 1,
			0,-d, 0, d,
			-d, 0, d, 0,
			c,-c, c,-c;
	plus_type_trans_inv_ = plus_type_trans_.inverse();

	x_type_trans_ << 1, 1, 1, 1,
			std::sqrt(2.0)*d/2.0,-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			c,-c, c,-c;
	x_type_trans_inv_ = x_type_trans_.inverse();
}

QuadCmd QuadMixer::CalcMotorCmd(float force, float torque[3], QuadFlightType type)
{
    Eigen::Matrix<float,4,1> force_torque;

    force_torque << force, torque[0], torque[1], torque[2];

    auto ang_vel = CalcMotorCmd(force_torque, type);

	QuadCmd cmd;

	cmd.ang_vel[0] = ang_vel(0);
	cmd.ang_vel[1] = ang_vel(1);
	cmd.ang_vel[2] = ang_vel(2);
	cmd.ang_vel[3] = ang_vel(3);

	return cmd;
}

Eigen::Matrix<double,4,1> QuadMixer::CalcMotorCmd(Eigen::Matrix<float,4,1> force_torque, QuadFlightType type)
{
    Eigen::Matrix<double,4,4> trans_inv;
	Eigen::Matrix<double,4,1> f_motor;
	Eigen::Matrix<double,4,1> ang_vel;

	if(type == QuadFlightType::PLUS_TYPE) {
		trans_inv = plus_type_trans_inv_;
	}
	else if(type == QuadFlightType::X_TYPE) {
		trans_inv = x_type_trans_inv_;
	}

	f_motor(0) = trans_inv(0,0) * force_torque(0) + trans_inv(0,1) * force_torque(1) + trans_inv(0,2) * force_torque(2) + trans_inv(0,3) * force_torque(3);
	f_motor(1) = trans_inv(1,0) * force_torque(0) + trans_inv(1,1) * force_torque(1) + trans_inv(1,2) * force_torque(2) + trans_inv(1,3) * force_torque(3);
	f_motor(2) = trans_inv(2,0) * force_torque(0) + trans_inv(2,1) * force_torque(1) + trans_inv(2,2) * force_torque(2) + trans_inv(2,3) * force_torque(3);
	f_motor(3) = trans_inv(3,0) * force_torque(0) + trans_inv(3,1) * force_torque(1) + trans_inv(3,2) * force_torque(2) + trans_inv(3,3) * force_torque(3);

	ang_vel = f_motor/state_.kF_;

	for(int i = 0; i < 4; i++) {
		if(ang_vel(i) < 0)
			ang_vel(i) = 0;
		ang_vel(i) = std::sqrt(ang_vel(i));
	}

	return ang_vel;
}

