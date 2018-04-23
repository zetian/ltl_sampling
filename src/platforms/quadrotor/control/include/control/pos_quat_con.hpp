/* 
 * pos_quat_con.hpp
 * 
 * Created on: Mar 3, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef POS_QUAT_CON_HPP
#define POS_QUAT_CON_HPP

#include "interface/controller_if.hpp"
#include "control/quad_state.hpp"

namespace librav {

struct PosQuatConParam
{
	float kp_0;
	float ki_0;
	float kd_0;
	float kp_1;
	float ki_1;
	float kd_1;
	float kp_2;
	float ki_2;
	float kd_2;

	double zint_uppper_limit = 0.1;
	double zint_lower_limit = -1.0;
	double xyint_uppper_limit = 0.8;
	double xyint_lower_limit = -0.8;

	double ts_ = 0.01;
};

struct PosQuatConInput
{
	// input of position controller
	float pos_d[3];
	float vel_d[3];
	float acc_d[3];
	float jerk_d[3];
	float yaw_d;
	float yaw_rate_d;
};

struct PosQuatConOutput{
	// output of position controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float rot_rate_d[3];
	float ftotal_d;
};

using PosQuatConIF = ControllerInterface<PosQuatConParam, QuadState,PosQuatConInput,PosQuatConOutput>;

class PosQuatCon: public PosQuatConIF
{
public:
	PosQuatCon(const QuadState& _rs);
	~PosQuatCon() = default;

private:
	double pos_e_integral[3];
	double last_acc_desired_[3];

public:
	void InitParams(const PosQuatConParam& param) override;
	void Update(const PosQuatConInput& desired, PosQuatConOutput* cmd) override;
};

}

#endif /* POS_QUAT_CON_HPP */
