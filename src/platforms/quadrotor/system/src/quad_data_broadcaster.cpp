/* 
 * quad_data_broadcaster.cpp
 * 
 * Created on: May 26, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <iostream>

#include "lcmtypes/librav.hpp"
#include "system/quad_data_broadcaster.hpp"

using namespace librav;

QuadDataBroadcaster::QuadDataBroadcaster(std::shared_ptr<lcm::LCM> lcm_ptr):
		lcm_(lcm_ptr)
{
	if(!lcm_->good())
		std::cerr << "LCM instance is not initialized properly. Quad state broadcaster is not going to work." << std::endl;
}

void QuadDataBroadcaster::SendQuadStateData(const QuadState& rs)
{
	SendLaserPoints(rs.laser_points_,rs.position_, rs.quat_);
	SendQuadTransform(rs.position_, rs.quat_);
}

void QuadDataBroadcaster::SendQuadTransform(Point3f pos, Eigen::Quaterniond quat)
{
	srcl_lcm_msgs::QuadrotorTransform trans_msg;
	srcl_lcm_msgs::Pose_t trans_base2world;
	srcl_lcm_msgs::Pose_t trans_laser2base;

	trans_base2world.position[0] = pos.x;
	trans_base2world.position[1] = pos.y;
	trans_base2world.position[2] = pos.z;

	trans_base2world.quaternion[0] = quat.w();
	trans_base2world.quaternion[1] = quat.x();
	trans_base2world.quaternion[2] = quat.y();
	trans_base2world.quaternion[3] = quat.z();

	trans_laser2base.position[0] = 0;
	trans_laser2base.position[1] = 0;
	trans_laser2base.position[2] = 0.11;

	trans_laser2base.quaternion[0] = 1.0;
	trans_laser2base.quaternion[1] = 0;
	trans_laser2base.quaternion[2] = 0;
	trans_laser2base.quaternion[3] = 0;

	trans_msg.base_to_world = trans_base2world;
	trans_msg.laser_to_base = trans_laser2base;

	lcm_->publish("quad_data/quad_transform", &trans_msg);
}

void QuadDataBroadcaster::SendLaserPoints(const std::vector<Point3f>& pts)
{
	srcl_lcm_msgs::LaserScanPoints_t pts_msg;
	srcl_lcm_msgs::Point3Df_t point;

	// assign values to msg
	for(auto& pt:pts)
	{
		point.x = pt.x;
		point.y = pt.y;
		point.z = pt.z;

		pts_msg.points.push_back(point);
	}
	pts_msg.point_num = pts_msg.points.size();

	lcm_->publish("quad_data/laser_scan_points", &pts_msg);
}

void QuadDataBroadcaster::SendLaserPoints(const std::vector<Point3f>& pts, Point3f pos, Eigen::Quaterniond quat)
{
	srcl_lcm_msgs::LaserScanPoints_t pts_msg;
	srcl_lcm_msgs::Point3Df_t point;

	// assign values to msg
	for(auto& pt:pts)
	{
		point.x = pt.x;
		point.y = pt.y;
		point.z = pt.z;

		pts_msg.points.push_back(point);
	}
	pts_msg.point_num = pts_msg.points.size();

	srcl_lcm_msgs::Pose_t trans_base2world;
	srcl_lcm_msgs::Pose_t trans_laser2base;

	trans_base2world.position[0] = pos.x;
	trans_base2world.position[1] = pos.y;
	trans_base2world.position[2] = pos.z;

	trans_base2world.quaternion[0] = quat.w();
	trans_base2world.quaternion[1] = quat.x();
	trans_base2world.quaternion[2] = quat.y();
	trans_base2world.quaternion[3] = quat.z();

	trans_laser2base.position[0] = 0;
	trans_laser2base.position[1] = 0;
	trans_laser2base.position[2] = 0.11;

	trans_laser2base.quaternion[0] = 1.0;
	trans_laser2base.quaternion[1] = 0;
	trans_laser2base.quaternion[2] = 0;
	trans_laser2base.quaternion[3] = 0;

	pts_msg.pose.base_to_world = trans_base2world;
	pts_msg.pose.laser_to_base = trans_laser2base;

	lcm_->publish("quad_data/laser_scan_points", &pts_msg);
}

void QuadDataBroadcaster::SendSystemTime(uint64_t sys_t)
{
	srcl_lcm_msgs::TimeStamp_t t_msg;

	t_msg.time_stamp = sys_t;

	lcm_->publish("quad_data/system_time", &t_msg);
}
