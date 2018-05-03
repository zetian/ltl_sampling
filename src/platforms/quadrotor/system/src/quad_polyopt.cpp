/* 
 * quad_polyopt.cpp
 * 
 * Created on: Aug 29, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <ctime>

#include "system/quad_polyopt.hpp"
#include "polyopt/polyopt_math.h"

using namespace librav;

QuadPolyOpt::QuadPolyOpt():
		r_pos_(4), N_pos_(10),
		r_yaw_(2), N_yaw_(3),
		keyframe_num_(2)
{

}

QuadPolyOpt::~QuadPolyOpt()
{

}

void QuadPolyOpt::InitOptMatrices(uint32_t keyframe_num)
{
//	keyframe_x_vals_ = Eigen::MatrixXf::Ones(r_pos_, keyframe_num) * std::numeric_limits<float>::infinity();
//	keyframe_y_vals_ = Eigen::MatrixXf::Ones(r_pos_, keyframe_num) * std::numeric_limits<float>::infinity();
//	keyframe_z_vals_ = Eigen::MatrixXf::Ones(r_pos_, keyframe_num) * std::numeric_limits<float>::infinity();
//	keyframe_yaw_vals_ = Eigen::MatrixXf::Ones(r_yaw_, keyframe_num) * std::numeric_limits<float>::infinity();

	keyframe_num_ = keyframe_num;

	keyframe_x_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_y_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_z_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_yaw_vals_ = Eigen::MatrixXf::Zero(r_yaw_, keyframe_num);

	keyframe_ts_ = Eigen::MatrixXf::Zero(1, keyframe_num);
}

void QuadPolyOpt::OptimizeFlatTraj()
{
	std::cout << "optimization called" << std::endl;

	clock_t exec_time;
	exec_time = clock();

	traj_[0] = optimizer_.OptimizeTrajectory(keyframe_x_vals_, keyframe_ts_, keyframe_num_, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[1] = optimizer_.OptimizeTrajectory(keyframe_y_vals_, keyframe_ts_, keyframe_num_, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[2] = optimizer_.OptimizeTrajectory(keyframe_z_vals_, keyframe_ts_, keyframe_num_, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[3] = optimizer_.OptimizeTrajectory(keyframe_yaw_vals_, keyframe_ts_, keyframe_num_, N_yaw_, r_yaw_);

	exec_time = clock() - exec_time;
	std::cout << "All optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

	for(int i = 0; i < traj_[0].segments.size(); i++)
	{
		std::vector<std::vector<double>> segcoeffs;
		double ts,te;
		for(int j = 0; j < 4; j++)
			segcoeffs.push_back(traj_[j].segments[i].coeffs);

		ts = traj_[0].segments[i].ts;
		te = traj_[0].segments[i].te;
		flat_traj_.AddTrajSeg(segcoeffs, ts, te);
	}

	std::cout << "Optimization result copied to flat_traj_.\n" << std::endl;
}

void QuadPolyOpt::OptimizeFlatTraj(const Eigen::Ref<const Eigen::MatrixXf> keyframe_x_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_y_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_z_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_yaw_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
			uint32_t keyframe_num)
{
	std::cout << "optimization called" << std::endl;

	clock_t exec_time;
	exec_time = clock();

	traj_[0] = optimizer_.OptimizeTrajectory(keyframe_x_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[1] = optimizer_.OptimizeTrajectory(keyframe_y_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[2] = optimizer_.OptimizeTrajectory(keyframe_z_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[3] = optimizer_.OptimizeTrajectory(keyframe_yaw_vals, keyframe_ts, keyframe_num, N_yaw_, r_yaw_);

	exec_time = clock() - exec_time;
	std::cout << "All optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

	for(int i = 0; i < traj_[0].segments.size(); i++)
	{
		std::vector<std::vector<double>> segcoeffs;
		double ts,te;
		for(int j = 0; j < 4; j++)
			segcoeffs.push_back(traj_[j].segments[i].coeffs);

		ts = traj_[0].segments[i].ts;
		te = traj_[0].segments[i].te;
		flat_traj_.AddTrajSeg(segcoeffs, ts, te);
	}

	std::cout << "Optimization result copied to flat_traj_.\n" << std::endl;
}

void QuadPolyOpt::InitOptJointMatrices(uint32_t keyframe_num)
{
	keyframe_num_ = keyframe_num;

	keyframe_x_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_y_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_z_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_yaw_vals_ = Eigen::MatrixXf::Zero(r_yaw_, keyframe_num);

	opt_size_.Qpos_size = (keyframe_num - 1) * (N_pos_ + 1);
	opt_size_.Qyaw_size = (keyframe_num - 1) * (N_yaw_ + 1);

	Q_x_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);
	Q_y_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);
	Q_z_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);
	Q_yaw_ = Eigen::MatrixXf::Zero(opt_size_.Qyaw_size, opt_size_.Qyaw_size);

	opt_size_.Aeq_pos_size_r = (keyframe_num - 1) * 2 * r_pos_;
	opt_size_.Aeq_pos_size_c = (keyframe_num - 1) * (N_pos_ + 1);
	opt_size_.Aeq_yaw_size_r = (keyframe_num - 1) * 2 * r_yaw_;
	opt_size_.Aeq_yaw_size_c = (keyframe_num - 1) * (N_yaw_ + 1);

	Aeq_x_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);
	Aeq_y_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);
	Aeq_z_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);
	Aeq_yaw_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_yaw_size_r, opt_size_.Aeq_yaw_size_c);

	opt_size_.beq_pos_size = (keyframe_num - 1) * 2 * r_pos_;
	opt_size_.beq_yaw_size = (keyframe_num - 1) * 2 * r_yaw_;

	beq_x_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);
	beq_y_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);
	beq_z_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);
	beq_yaw_ = Eigen::MatrixXf::Zero(opt_size_.beq_yaw_size, 1);

	Q_joint_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size * 3 + opt_size_.Qyaw_size, opt_size_.Qpos_size * 3 + opt_size_.Qyaw_size);
	Aeq_joint_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r * 3 + opt_size_.Aeq_yaw_size_r, opt_size_.Aeq_pos_size_c * 3 + opt_size_.Aeq_yaw_size_c);
	beq_joint_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size * 3 + opt_size_.beq_yaw_size, 1);

	keyframe_ts_ = Eigen::MatrixXf::Zero(1, keyframe_num);

	opt_size_.var_size = (N_pos_ + 1)*(keyframe_num - 1)*3 + (N_yaw_ + 1)*(keyframe_num - 1);
}

void QuadPolyOpt::OptimizeFlatTrajJoint()
{
	std::cout << "optimization called" << std::endl;

	clock_t exec_time;
	exec_time = clock();

	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_x_);
	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_y_);
	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_z_);
	PolyOptMath::GetNonDimQMatrices(N_yaw_,r_yaw_,keyframe_num_, keyframe_ts_, Q_yaw_);

	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_x_vals_, keyframe_ts_, Aeq_x_, beq_x_);
	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_y_vals_, keyframe_ts_, Aeq_y_, beq_y_);
	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_z_vals_, keyframe_ts_, Aeq_z_, beq_z_);
	PolyOptMath::GetNonDimEqualityConstrs(N_yaw_, r_yaw_, keyframe_num_, keyframe_yaw_vals_, keyframe_ts_, Aeq_yaw_, beq_yaw_);

	Q_joint_.block(0, 0, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_x_;
	Q_joint_.block(opt_size_.Qpos_size, opt_size_.Qpos_size, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_y_;
	Q_joint_.block(opt_size_.Qpos_size*2, opt_size_.Qpos_size*2, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_z_;
	Q_joint_.block(opt_size_.Qpos_size*3, opt_size_.Qpos_size*3, opt_size_.Qyaw_size, opt_size_.Qyaw_size) = Q_yaw_;

	Aeq_joint_.block(0 , 0, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_x_;
	Aeq_joint_.block(opt_size_.Aeq_pos_size_r , opt_size_.Aeq_pos_size_c, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_y_;
	Aeq_joint_.block(opt_size_.Aeq_pos_size_r*2 , opt_size_.Aeq_pos_size_c*2, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_z_;
	Aeq_joint_.block(opt_size_.Aeq_pos_size_r*3 , opt_size_.Aeq_pos_size_c*3, opt_size_.Aeq_yaw_size_r, opt_size_.Aeq_yaw_size_c) = Aeq_yaw_;

	beq_joint_.block(0 , 0, opt_size_.beq_pos_size, 1) = beq_x_;
	beq_joint_.block(opt_size_.beq_pos_size , 0, opt_size_.beq_pos_size, 1) = beq_y_;
	beq_joint_.block(opt_size_.beq_pos_size*2 , 0, opt_size_.beq_pos_size, 1) = beq_z_;
	beq_joint_.block(opt_size_.beq_pos_size*3 , 0, opt_size_.beq_yaw_size, 1) = beq_yaw_;

	OptResultParam result = optimizer_.OptimizeTrajectory(Q_joint_, Aeq_joint_, beq_joint_, keyframe_ts_, keyframe_num_, opt_size_.var_size);

	exec_time = clock() - exec_time;
	std::cout << "All optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

//	for(auto& param:result.params)
//		std::cout << param << std::endl;

	if(result.params.empty())
	{
		std::cerr << "Bad optimization result." << std::endl;

		return;
	}

	// clear results from last optimization call
	for(int dim = 0; dim < 4; dim++)
		traj_[dim].segments.clear();

	// copy result to dimension: x, y , z
	for(int dim = 0; dim < 3; dim++)
	{
		for(int seg = 0; seg < keyframe_num_ - 1; seg++)
		{
			CurveParameter seg_param;

			for(int i = 0; i < N_pos_ + 1; i++)
				seg_param.coeffs.push_back(result.params[dim*(keyframe_num_ - 1)*(N_pos_ + 1) + seg * (N_pos_ + 1) + i]);

			seg_param.ts = keyframe_ts_(0, seg);
			seg_param.te = keyframe_ts_(0, seg+1);

			traj_[dim].segments.push_back(seg_param);
		}
	}

	// copy result to dimension: yaw
	for(int seg = 0; seg < keyframe_num_ - 1; seg++)
	{
		CurveParameter seg_param;

		for(int i = 0; i < N_yaw_ + 1; i++) {
			//std::cout << "index: " << 3*(N_pos_ + 1)*(keyframe_num_ - 1) + seg * (N_yaw_ + 1) + i << std::endl;
			seg_param.coeffs.push_back(result.params[3*(N_pos_ + 1)*(keyframe_num_ - 1) + seg * (N_yaw_ + 1) + i]);
		}
		seg_param.ts = keyframe_ts_(0, seg);
		seg_param.te = keyframe_ts_(0, seg+1);
		traj_[3].segments.push_back(seg_param);
	}

	// copy result to flat_traj_
	for(int i = 0; i < traj_[0].segments.size(); i++)
	{
		std::vector<std::vector<double>> segcoeffs;
		double ts,te;
		for(int j = 0; j < 4; j++)
			segcoeffs.push_back(traj_[j].segments[i].coeffs);

		ts = traj_[0].segments[i].ts;
		te = traj_[0].segments[i].te;
		flat_traj_.AddTrajSeg(segcoeffs, ts, te);
	}

	std::cout << "Optimization result copied to flat_traj_.\n" << std::endl;
}

void QuadPolyOpt::InitOptWithCorridorJointMatrices(uint32_t keyframe_num, uint32_t midpoint_num, double cor_size)
{
	std::cout << "started init" << std::endl;
	keyframe_num_ = keyframe_num;

	// position constraints
	keyframe_x_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_y_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);
	keyframe_z_vals_ = Eigen::MatrixXf::Zero(r_pos_, keyframe_num);

	opt_size_.Qpos_size = (keyframe_num - 1) * (N_pos_ + 1);
	Q_x_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);
	Q_y_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);
	Q_z_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size, opt_size_.Qpos_size);

	opt_size_.Aeq_pos_size_r = (keyframe_num - 1) * 2 * r_pos_;
	opt_size_.Aeq_pos_size_c = (keyframe_num - 1) * (N_pos_ + 1);
	Aeq_x_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);
	Aeq_y_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);
	Aeq_z_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c);

	opt_size_.beq_pos_size = (keyframe_num - 1) * 2 * r_pos_;
	beq_x_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);
	beq_y_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);
	beq_z_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size, 1);

	Q_joint_ = Eigen::MatrixXf::Zero(opt_size_.Qpos_size * 3, opt_size_.Qpos_size * 3);
	Aeq_joint_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_pos_size_r * 3, opt_size_.Aeq_pos_size_c * 3);
	beq_joint_ = Eigen::MatrixXf::Zero(opt_size_.beq_pos_size * 3, 1);

	// corridor constraints
	opt_size_.cor_constr_size = midpoint_num * 2 * 3 * (keyframe_num_ - 1);
	A_cor_ =  Eigen::MatrixXf::Zero(opt_size_.cor_constr_size, (N_pos_ + 1) * 3 * (keyframe_num_ - 1));
	b_cor_ =  Eigen::MatrixXf::Zero(opt_size_.cor_constr_size, 1);

	// yaw constraints
	keyframe_yaw_vals_ = Eigen::MatrixXf::Zero(r_yaw_, keyframe_num);

	opt_size_.Qyaw_size = (keyframe_num - 1) * (N_yaw_ + 1);
	Q_yaw_ = Eigen::MatrixXf::Zero(opt_size_.Qyaw_size, opt_size_.Qyaw_size);

	opt_size_.Aeq_yaw_size_r = (keyframe_num - 1) * 2 * r_yaw_;
	opt_size_.Aeq_yaw_size_c = (keyframe_num - 1) * (N_yaw_ + 1);
	Aeq_yaw_ = Eigen::MatrixXf::Zero(opt_size_.Aeq_yaw_size_r, opt_size_.Aeq_yaw_size_c);

	opt_size_.beq_yaw_size = (keyframe_num - 1) * 2 * r_yaw_;
	beq_yaw_ = Eigen::MatrixXf::Zero(opt_size_.beq_yaw_size, 1);

	// time constraints
	keyframe_ts_ = Eigen::MatrixXf::Zero(1, keyframe_num);

	// optimization variable size
	opt_size_.midpoint_num = midpoint_num;
	opt_size_.corridor_size = cor_size;
	opt_size_.var_cor_pos_size = (N_pos_ + 1)*(keyframe_num - 1)*3;
	opt_size_.var_cor_yaw_size = (N_yaw_ + 1)*(keyframe_num - 1);

	std::cout << "ended init" << std::endl;
}

bool QuadPolyOpt::OptimizeFlatTrajWithCorridorJoint()
{
	std::cout << "optimization called" << std::endl;

	clock_t exec_time;
	exec_time = clock();

	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_x_);
	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_y_);
	PolyOptMath::GetNonDimQMatrices(N_pos_,r_pos_,keyframe_num_, keyframe_ts_, Q_z_);
	PolyOptMath::GetNonDimQMatrices(N_yaw_,r_yaw_,keyframe_num_, keyframe_ts_, Q_yaw_);

	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_x_vals_, keyframe_ts_, Aeq_x_, beq_x_);
	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_y_vals_, keyframe_ts_, Aeq_y_, beq_y_);
	PolyOptMath::GetNonDimEqualityConstrs(N_pos_, r_pos_, keyframe_num_, keyframe_z_vals_, keyframe_ts_, Aeq_z_, beq_z_);
	PolyOptMath::GetNonDimEqualityConstrs(N_yaw_, r_yaw_, keyframe_num_, keyframe_yaw_vals_, keyframe_ts_, Aeq_yaw_, beq_yaw_);

	corridor_frames_.clear();
	corridor_frames_.push_back(keyframe_x_vals_);
	corridor_frames_.push_back(keyframe_y_vals_);
	corridor_frames_.push_back(keyframe_z_vals_);
	PolyOptMath::GetNonDimCorridorConstrs(N_pos_, keyframe_num_, opt_size_.midpoint_num, opt_size_.corridor_size,
			corridor_frames_, keyframe_ts_, A_cor_, b_cor_);

	Q_joint_.block(0, 0, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_x_;
	Q_joint_.block(opt_size_.Qpos_size, opt_size_.Qpos_size, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_y_;
	Q_joint_.block(opt_size_.Qpos_size*2, opt_size_.Qpos_size*2, opt_size_.Qpos_size, opt_size_.Qpos_size) = Q_z_;

	Aeq_joint_.block(0 , 0, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_x_;
	Aeq_joint_.block(opt_size_.Aeq_pos_size_r , opt_size_.Aeq_pos_size_c, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_y_;
	Aeq_joint_.block(opt_size_.Aeq_pos_size_r*2 , opt_size_.Aeq_pos_size_c*2, opt_size_.Aeq_pos_size_r, opt_size_.Aeq_pos_size_c) = Aeq_z_;

	beq_joint_.block(0 , 0, opt_size_.beq_pos_size, 1) = beq_x_;
	beq_joint_.block(opt_size_.beq_pos_size , 0, opt_size_.beq_pos_size, 1) = beq_y_;
	beq_joint_.block(opt_size_.beq_pos_size*2 , 0, opt_size_.beq_pos_size, 1) = beq_z_;

	OptResultParam pos_result = optimizer_.OptimizeTrajectory(Q_joint_, Aeq_joint_, beq_joint_, A_cor_, b_cor_, keyframe_ts_, keyframe_num_, opt_size_.var_cor_pos_size);
	OptResultParam yaw_result = optimizer_.OptimizeTrajectory(Q_yaw_, Aeq_yaw_, beq_yaw_, keyframe_ts_, keyframe_num_, opt_size_.var_cor_yaw_size);

	exec_time = clock() - exec_time;
	std::cout << "All optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

//	std::cout << "pos param: " << std::endl;
//	for(auto& param:pos_result.params)
//		std::cout << param << std::endl;

	if(pos_result.params.empty())// || yaw_result.params.empty())
	{
		std::cerr << "Bad optimization result." << std::endl;

		return false;
	}

	// clear results from last optimization call
	for(int dim = 0; dim < 4; dim++)
		traj_[dim].segments.clear();

	// copy result to dimension: x, y , z
	for(int dim = 0; dim < 3; dim++)
	{
		for(int seg = 0; seg < keyframe_num_ - 1; seg++)
		{
			CurveParameter seg_param;

			for(int i = 0; i < N_pos_ + 1; i++)
				seg_param.coeffs.push_back(pos_result.params[dim*(keyframe_num_ - 1)*(N_pos_ + 1) + seg * (N_pos_ + 1) + i]);

			seg_param.ts = keyframe_ts_(0, seg);
			seg_param.te = keyframe_ts_(0, seg+1);

			traj_[dim].segments.push_back(seg_param);
		}
	}

	// copy result to dimension: yaw
	for(int seg = 0; seg < keyframe_num_ - 1; seg++)
	{
		CurveParameter seg_param;

		for(int i = 0; i < N_yaw_ + 1; i++) {
			//std::cout << "index: " << 3*(N_pos_ + 1)*(keyframe_num_ - 1) + seg * (N_yaw_ + 1) + i << std::endl;
			seg_param.coeffs.push_back(yaw_result.params[seg * (N_yaw_ + 1) + i]);
		}
		seg_param.ts = keyframe_ts_(0, seg);
		seg_param.te = keyframe_ts_(0, seg+1);
		traj_[3].segments.push_back(seg_param);
	}

	// copy result to flat_traj_
	for(int i = 0; i < traj_[0].segments.size(); i++)
	{
		std::vector<std::vector<double>> segcoeffs;
		double ts,te;
		for(int j = 0; j < 4; j++)
			segcoeffs.push_back(traj_[j].segments[i].coeffs);

		ts = traj_[0].segments[i].ts;
		te = traj_[0].segments[i].te;
		flat_traj_.AddTrajSeg(segcoeffs, ts, te);
	}

	std::cout << "Optimization result copied to flat_traj_.\n" << std::endl;

	return true;
}

