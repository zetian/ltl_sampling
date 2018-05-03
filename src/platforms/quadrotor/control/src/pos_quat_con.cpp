/* 
 * pos_quat_con.cpp
 * 
 * Created on: Mar 3, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "logging/logger.hpp"
#include "control/pos_quat_con.hpp"

using namespace librav;

PosQuatCon::PosQuatCon(const QuadState& _rs):
		PosQuatConIF(_rs)
{
	// 0-1: 3.8, 0.08, 3.2
	param_.kp_0 = 3.8;
	param_.ki_0 = 0.1;
	param_.kd_0 = 3.2;

	param_.kp_1 = 3.8;
	param_.ki_1 = 0.1;
	param_.kd_1 = 3.2;

	// kp kd 1.8 2.45
//	kp_2 = 1.2;
//	ki_2 = 0.08;
//	kd_2 = 1.85;
	// 1.25, 0.145, 1.65
	// 1.8, 0.05, 1.85
	param_.kp_2 = 1.8;
	param_.ki_2 = 0.05;
	param_.kd_2 = 1.85;

	param_.zint_uppper_limit = 0.1;
	param_.zint_lower_limit = -1.0;
	param_.xyint_uppper_limit = 0.8;
	param_.xyint_lower_limit = -0.8;

	param_.ts_ = 0.01;

	for(int i = 0; i < 3; i++)
	{
		pos_e_integral[i] = 0.0;
		last_acc_desired_[i] = 0.0;
	}
}

void PosQuatCon::InitParams(const PosQuatConParam& param)
{
	param_ = param;

	for(int i = 0; i < 3; i++)
	{
		pos_e_integral[i] = 0.0;
		last_acc_desired_[i] = 0.0;
	}

	initialized_ = true;
}

void PosQuatCon::Update(const PosQuatConInput& input, PosQuatConOutput* output)
{
	if(!initialized_)
		return;

	float pos_error[3],vel_error[3];

	pos_error[0] = input.pos_d[0] - state_.position_.x;
	pos_error[1] = input.pos_d[1] - state_.position_.y;
	pos_error[2] = input.pos_d[2] - state_.position_.z;

	vel_error[0] = input.vel_d[0] - state_.velocity_.x;
	vel_error[1] = input.vel_d[1] - state_.velocity_.y;
	vel_error[2] = input.vel_d[2] - state_.velocity_.z;

	for(int i = 0; i < 3; i++)
	{
		if(pos_error[i] < 0.002 && pos_error[i] > -0.002)
			pos_error[i] = 0;
		if(vel_error[i] < 0.002 && vel_error[i] > -0.002)
			vel_error[i] = 0;
	}

	pos_e_integral[0] = pos_e_integral[0] + pos_error[0];
	pos_e_integral[1] = pos_e_integral[1] + pos_error[1];
	pos_e_integral[2] = pos_e_integral[2] + pos_error[2];

	double ri_acc_fb[3];

	ri_acc_fb[0] = param_.kp_0 * pos_error[0] + param_.ki_0 * pos_e_integral[0] + param_.kd_0 * vel_error[0];
	ri_acc_fb[1] = param_.kp_1 * pos_error[1] + param_.ki_1 * pos_e_integral[1] + param_.kd_1 * vel_error[1];
	ri_acc_fb[2] = param_.kp_2 * pos_error[2] + param_.ki_2 * pos_e_integral[2] + param_.kd_2 * vel_error[2];

	if(pos_e_integral[0] > param_.xyint_uppper_limit)
		pos_e_integral[0] = param_.xyint_uppper_limit;
	if(pos_e_integral[0] < param_.xyint_lower_limit)
		pos_e_integral[0] = param_.xyint_lower_limit;

	if(pos_e_integral[1] > param_.xyint_uppper_limit)
		pos_e_integral[1] = param_.xyint_uppper_limit;
	if(pos_e_integral[1] < param_.xyint_lower_limit)
		pos_e_integral[1] = param_.xyint_lower_limit;

	if(pos_e_integral[2] > param_.zint_uppper_limit)
		pos_e_integral[2] = param_.zint_uppper_limit;
	if(pos_e_integral[2] < param_.zint_lower_limit)
		pos_e_integral[2] = param_.zint_lower_limit;

	Eigen::Vector3d Fi;
	Eigen::Vector3d Fi_n;
	Eigen::Vector3d Fb_n(0,0,1);

	Fi = state_.mass_ * Eigen::Vector3d(ri_acc_fb[0]+input.acc_d[0], ri_acc_fb[1]+input.acc_d[1], ri_acc_fb[2] + input.acc_d[2] + state_.g_);
	Fi_n = Fi.normalized();

	Eigen::Vector4d qd_n;
	Eigen::Vector3d FbFi_cross;
	double FbT_Fi;
	double scale;
	double qd_wd;
	Eigen::Quaterniond quat_pr;

	FbT_Fi = Fb_n.transpose() * Fi_n;

	if(FbT_Fi < 0)
	{
		Fb_n(2) = -1;
		FbT_Fi = Fb_n.transpose() * Fi_n;
	}

	qd_wd = 1+FbT_Fi;
	scale = 1/sqrt(2*qd_wd);
	FbFi_cross = Fb_n.cross(Fi_n);

	quat_pr.w() = qd_wd/scale;
	quat_pr.x() = FbFi_cross(0)/scale;
	quat_pr.y() = FbFi_cross(1)/scale;
	quat_pr.z() = FbFi_cross(2)/scale;

//	quat_y.w() = cos(input->yaw_d/2);
//	quat_y.x() = 0;
//	quat_y.y() = 0;
//	quat_y.z() = sin(input->yaw_d/2);

	quat_pr.normalize();
	Eigen::Quaterniond quat_y(Eigen::AngleAxisd(input.yaw_d, quat_pr.matrix().col(2)));
	Eigen::Quaterniond quat_result = quat_y * quat_pr;

	output->quat_d = quat_result.normalized();
	output->ftotal_d = Fi.norm();

	float omega_d[3];
	Eigen::Vector3d Fidot;
	Eigen::Vector3d ri_jerk_d(input.jerk_d[0], input.jerk_d[1], input.jerk_d[2]);
	Eigen::Vector3d ri_jerk_fb;

	ri_jerk_fb = Eigen::Vector3d(ri_acc_fb[0]-last_acc_desired_[0],
			ri_acc_fb[1]-last_acc_desired_[1], ri_acc_fb[2]-last_acc_desired_[2]) / param_.ts_;
	Fidot = state_.mass_ * (ri_jerk_d + ri_jerk_fb);

	Eigen::Vector3d Fidot_n;
	Fidot_n = Fidot / Fi.norm() - Fi * (Fi.transpose() * Fidot)/std::pow(Fi.norm(), 3);

	Eigen::Vector3d omega_dxy;

	omega_dxy = Fi_n.cross(Fidot_n);

	// convert omega_dxy from inertia frame to body frame
	Eigen::Vector3d omega_dxy_b;
	Eigen::Quaterniond p, rotated_p;
	p.w() = 0;
	p.vec() = omega_dxy;
	rotated_p = state_.quat_ * p * state_.quat_.inverse();
	omega_dxy_b = rotated_p.vec();

	omega_d[0] = omega_dxy_b(0);
	omega_d[1] = omega_dxy_b(1);
	omega_d[2] = input.yaw_rate_d;

	output->rot_rate_d[0] = omega_d[0];
	output->rot_rate_d[1] = omega_d[1];
	output->rot_rate_d[2] = omega_d[2];

	// set control values to be zero if too small
	if(output->quat_d.x() < 10e-6 && output->quat_d.x() > -10e-6)
		output->quat_d.x() = 0;
	if(output->quat_d.y() < 10e-6 && output->quat_d.y() > -10e-6)
		output->quat_d.y() = 0;
	if(output->quat_d.z() < 10e-6 && output->quat_d.z() > -10e-6)
		output->quat_d.z() = 0;
	if(output->quat_d.w() < 10e-6 && output->quat_d.w() > -10e-6)
		output->quat_d.w() = 0;

	for(int i = 0; i < 3; i++)
	{
		if(output->rot_rate_d[i] < 10e-6 && output->rot_rate_d[i] > -10e-6)
			output->rot_rate_d[i] = 0;

		// save values for next iteration
		last_acc_desired_[i] = ri_acc_fb[i];
	}

	CtrlLogger::GetLogger().AddItemDataToEntry("omega_d_x", output->rot_rate_d[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("omega_d_y", output->rot_rate_d[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("omega_d_z", output->rot_rate_d[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("pos_e_x", pos_error[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_e_y", pos_error[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_e_z", pos_error[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("vel_e_x", vel_error[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_e_y", vel_error[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_e_z", vel_error[2]);
}
