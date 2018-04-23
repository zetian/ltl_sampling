/* 
 * quad_state.cpp
 * 
 * Created on: Mar 2, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <cmath>
#include <iostream>
#include <iomanip>

#include "control/quad_state.hpp"

using namespace librav;

QuadState::QuadState():
		g_(9.8),
		mass_(0.57375),
		arm_length_(0.175),
		kF_(6.11e-8),
		kM_(1.5e-9),
		sim_step_(0.01),
		quad_flight_type_(QuadFlightType::X_TYPE)
{
	w_h_ = sqrt(mass_ * g_ / 4 / kF_);

	// get values directly from simulator, will be replaced by state estimator later
	position_.x = 0;
	position_.y = 0;
	position_.z = 0.04;

	velocity_.x = 0;
	velocity_.y = 0;
	velocity_.z = 0;

	// euler in X-Y-Z convension
	orientation_.x = 0;
	orientation_.y = 0;
	orientation_.z = 0;

	quat_.x() = 0;
	quat_.y() = 0;
	quat_.z() = 0;
	quat_.w() = 0;

	rotation_rate_.x = 0;
	rotation_rate_.y = 0;
	rotation_rate_.z = 0;
}

void QuadState::UpdateRobotState(const QuadSensorData& new_data)
{
	// get values directly from simulator, will be replaced by state estimator later
	position_.x = new_data.pos_i.x;
	position_.y = new_data.pos_i.y;
	position_.z = new_data.pos_i.z;

	velocity_.x = new_data.vel_i.x;
	velocity_.y = new_data.vel_i.y;
	velocity_.z = new_data.vel_i.z;

	// euler in X-Y-Z convension
	orientation_.x = new_data.rot_i.x;
	orientation_.y = new_data.rot_i.y;
	orientation_.z = new_data.rot_i.z;

	// quaternion
	quat_.x() = new_data.quat_i.x;
	quat_.y() = new_data.quat_i.y;
	quat_.z() = new_data.quat_i.z;
	quat_.w() = new_data.quat_i.w;

	rotation_rate_.x = new_data.rot_rate_b.x;
	rotation_rate_.y = new_data.rot_rate_b.y;
	rotation_rate_.z = new_data.rot_rate_b.z;

	laser_points_ = new_data.laser_points;
}
