/* 
 * att_quat_con.hpp
 * 
 * Created on: Mar 3, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef ATT_QUAT_CON_HPP
#define ATT_QUAT_CON_HPP

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "common/librav_types.hpp"
#include "interface/controller_if.hpp"
#include "control/quad_state.hpp"

namespace librav {

struct AttQuatConParam
{
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;
};

struct AttQuatConInput
{
	// quaternion
	Eigen::Quaterniond quat_d;
	// rotation rate
	float rot_rate_d[3];
	// total force
	// used to calculate motor commands
	float ftotal_d;
};

struct AttQuatConOutput
{
	float motor_ang_vel_d[4];
	float torque_d[3];
};

using AttQuatConIF = ControllerInterface<AttQuatConParam, QuadState,AttQuatConInput,AttQuatConOutput>;

class AttQuatCon: public AttQuatConIF
{
public:
	AttQuatCon(const QuadState& _rs);
	~AttQuatCon() = default;

private:
	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;

	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);

public:
	void InitParams(const AttQuatConParam& param) override;
	void Update(const AttQuatConInput& desired, AttQuatConOutput* output) override;
};

}

#endif /* ATT_QUAT_CON_HPP */
