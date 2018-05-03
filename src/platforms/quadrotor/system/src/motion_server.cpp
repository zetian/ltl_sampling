/* 
 * motion_server.cpp
 * 
 * Created on: May 23, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <cmath>
#include <iostream>

#include "system/motion_server.hpp"

using namespace librav;

//MotionServer::MotionServer():
//		goal_completed_(false),
//		ms_count_(0),
//		waypoint_idx_(0),
//		current_sys_time_(0),
//		mode_(MotionMode::POLYNOMIAL)
//{
////	UAVTrajectory test_traj = GenerateTestTrajectory();
////
////	SetMotionGoal(test_traj);
//}

MotionServer::MotionServer(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		polytraj_handler_(new QuadFlatTrajHandler(lcm_)),
		goal_completed_(false),
		ms_count_(0),
		waypoint_idx_(0),
		current_sys_time_(0),
		mode_(MotionMode::POLYNOMIAL)
{
	lcm_->subscribe("quad_data/system_time", &MotionServer::LcmSysTimeHandler, this);
	lcm_->subscribe("quad_controller/quad_motion_service", &MotionServer::LcmUserGoalHandler, this);
}

UAVTrajectory MotionServer::GenerateTestTrajectory()
{
	UAVTrajectory test_traj;

	int time_stamp1 = 50;
	int time_stamp2 = time_stamp1 + 150;
	int final_time_stamp = time_stamp2 + 10;

	for(int i = 0; i < final_time_stamp; i++)
	{
		UAVTrajectoryPoint pt;
		pt.point_empty = false;

		double height = 0.5;
		double radius = 1.0;
		double circle_ang_vel = 180.0/180.0*3.14;

		if(i < time_stamp1)
		{
			pt.positions[0] = 0;
			pt.positions[1] = 0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.yaw = 0;
			pt.duration = 1;
		}
		else if(i < time_stamp2)
		{
			double angle = (i - time_stamp1)*0.01*circle_ang_vel;
			pt.positions[0] = radius * cos(angle - M_PI/2);
			pt.positions[1] = radius * sin(angle - M_PI/2);
			pt.positions[2] = height;

			pt.velocities[0] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel;
			pt.velocities[1] = radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel;
			pt.velocities[2] = 0;

			pt.accelerations[0] = - radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[1] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[2] = 0;

			pt.yaw = angle;
			pt.duration = 5;
		}
		else
			pt.point_empty = true;

		test_traj.push_back(pt);
	}

	return test_traj;
}

void MotionServer::LcmSysTimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::TimeStamp_t* msg)
{
	current_sys_time_ = msg->time_stamp;
	polytraj_handler_->UpdateSystemTime(current_sys_time_);
}

void MotionServer::LcmUserGoalHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::UAVTrajectory_t* msg)
{
	std::cout << "motion service request received!" << std::endl;

	UAVTrajectory traj;

	for(int64_t i = 0; i < msg->waypoint_num; i++)
	{
		UAVTrajectoryPoint pt;

		pt.point_empty = false;

		for(int j = 0; j < 3; j++)
		{
			pt.positions[j] = msg->trajectory[i].positions[j];
			pt.velocities[j] = msg->trajectory[i].velocities[j];
			pt.accelerations[j] = msg->trajectory[i].accelerations[j];
			pt.jerks[j] = msg->trajectory[i].jerks[j];
		}
		pt.yaw = msg->trajectory[i].yaw;
		pt.yaw_rate = msg->trajectory[i].yaw_rate;
		pt.duration = msg->trajectory[i].duration;

		traj.push_back(pt);
	}

	if(!active_goal_.empty())
	{
		AbortActiveMotion();
		std::cout << "Old goal motion aborted!" << std::endl;
	}
	SetMotionGoal(traj);
}

void MotionServer::SetMotionGoal(UAVTrajectory& goal)
{
	active_goal_ = goal;
	goal_completed_  = false;
}

void MotionServer::AbortActiveMotion()
{
	SetGoalCompleted();
}

void MotionServer::SetGoalCompleted()
{
	waypoint_idx_ = 0;
	active_goal_.clear();
	goal_completed_ = true;

	std::cout << "task ended " << std::endl;
}

double MotionServer::ReportActiveMotionProgress()
{
	double percentage = 0;

	if(active_goal_.size() != 0)
		percentage = static_cast<double>(waypoint_idx_)/static_cast<double>(active_goal_.size());

	std::cout << "task completion: " << percentage << std::endl;

	return percentage;
}

void MotionServer::ReportProgress(void)
{
	switch(mode_)
	{
	case MotionMode::POLYNOMIAL:
		polytraj_handler_->ReportProgress();
		break;
	case MotionMode::WAYPOINTS:
	case MotionMode::USER_CMDS:
	case MotionMode::POS_STEP_RESPONSE:
	default:
		break;
	}
}

UAVTrajectoryPoint MotionServer::GetCurrentUserDefinedPose()
{
	UAVTrajectoryPoint pt;
	pt.point_empty = true;

	if(waypoint_idx_ < active_goal_.size())
	{
		if(ms_count_ < active_goal_[waypoint_idx_].duration)
		{
			pt = active_goal_[waypoint_idx_];
			ms_count_++;
		}
		else
		{
			waypoint_idx_++;
			ms_count_ = 1;

			if(waypoint_idx_ != active_goal_.size())
				pt = active_goal_[waypoint_idx_];
			else
			{
				SetGoalCompleted();
			}
		}

		if(!goal_completed_)
			ReportActiveMotionProgress();
	}

	return pt;
}

UAVTrajectoryPoint MotionServer::GetCurrentDesiredState(time_stamp t)
{
	UAVTrajectoryPoint pt;
	pt.point_empty = true;

	switch(mode_)
	{
	case MotionMode::POLYNOMIAL:
		return polytraj_handler_->GetDesiredTrajectoryPoint(t);
	case MotionMode::WAYPOINTS:
		return pt;
	case MotionMode::USER_CMDS:
		return GetCurrentUserDefinedPose();
	case MotionMode::POS_STEP_RESPONSE:
	{
		if(t < 1200)
		{
			pt.point_empty = false;
			pt.positions[0] = 0;
			pt.positions[1] = 0;
			pt.positions[2] = 0.5;
		}
		else
		{
			pt.point_empty = false;
			pt.positions[0] = 1;
			pt.positions[1] = 1;
			pt.positions[2] = 0.5;
		}
		return pt;
	}
	default:
		return pt;
	}
}
