/* 
 * path_manager.cpp
 * 
 * Created on: Oct 31, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "system/path_manager.hpp"

using namespace librav;

PathManager::PathManager(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		user_path_id_(0)
{
	//lcm_->subscribe("quad_planner/goal_waypoints",&TrajectoryGenerator::LcmWaypointsHandler, this);
	lcm_->subscribe("quad_planner/goal_keyframe_set",&PathManager::LcmKeyframeSetHandler, this);
}

std::vector<Position3Dd> PathManager::GetKeyTurningWaypoints(std::vector<Position3Dd>& wps)
{
	// first remove undesired points at the connections between 2d/3d geomarks
	std::vector<Position3Dd> smoothed_points = wps;
//	for(auto it = wps.begin(); it != wps.end() - 1; ++it)
//	{
//		Position3Dd pt1 = *it;
//		Position3Dd pt2 = *(it+1);
//
//		smoothed_points.push_back(pt1);
//
//		// fix the connections between 2d and 3d vertices
//		if(std::abs(pt1.z - pt2.z) > 0.05)
//		{
//			// ignore the last point if there is a 2d/3d connection
//			//	between the last two points
//			if(it + 2 != wps.end())
//			{
//				Position3Dd pt3 = *(it + 2);
//
//				Eigen::Vector3d v1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
//				Eigen::Vector3d v2(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z);
//
//				// skip next point if direction is opposite
//				if(v1.dot(v2) <= 0)
//					it++;
//			}
//		}
//		else if(it + 2 == wps.end())
//			smoothed_points.push_back(pt2);
//	}

	// std::cout << "selected points: " << smoothed_points.size() << std::endl;

	// then remove intermediate points in a straight line
	std::vector<Position3Dd> minimum_points;

	if(smoothed_points.size() <= 2)
	{
		minimum_points = smoothed_points;
	}
	else
	{
		// add first waypoint
		minimum_points.push_back(smoothed_points.front());
		Position3Dd last_wp = smoothed_points.front();
		// check intermediate waypoints
		for(int cid = 1; cid < smoothed_points.size() - 1; cid++)
		{
			Position3Dd pt1 = smoothed_points[cid - 1];
			Position3Dd pt2 = smoothed_points[cid];
			Position3Dd pt3 = smoothed_points[cid + 1];

			Eigen::Vector3d v1 = Eigen::Vector3d(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z).normalized();
			Eigen::Vector3d v2 = Eigen::Vector3d(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z).normalized();
			Eigen::Vector3d e = v1 - v2;

			double dist = std::sqrt(std::pow(smoothed_points[cid].x - last_wp.x,2) +
					std::pow(smoothed_points[cid].y - last_wp.y,2) +
					std::pow(smoothed_points[cid].z - last_wp.z,2));

			// |e| = sqrt[sin(theta)^2 + (1 - cos(theta))^2], |e| ~= 0.082 when theta = 5 degree
			if(e.norm() > 0.082 || dist > 0.2)
			{
				minimum_points.push_back(smoothed_points[cid]);
				last_wp = smoothed_points[cid];
			}
		}
		// add last waypoint
		minimum_points.push_back(smoothed_points.back());
	}

	return minimum_points;
	//return smoothed_points;
}

void PathManager::LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg)
{
	std::cout << "waypoints received: " << msg->waypoint_num << std::endl;

	// generate keyframes from waypoint
	KeyframeSet new_kfs;
	for(int i = 0; i < msg->waypoint_num; i++)
	{
		Keyframe kf;

		for(int j = 0; j < 3; j++)
			kf.position[j] = msg->waypoints[i].positions[j];

		kf.vel_constr = false;
		kf.yaw = msg->waypoints[i].yaw;
		new_kfs.keyframes.push_back(kf);
	}

	GenerateTrajectory(new_kfs, user_path_id_++);
}

void PathManager::LcmKeyframeSetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::KeyframeSet_t* msg)
{
	std::cout << "keyframes received: " << msg->kf_num << std::endl;

	// copy keyframes from msg
	KeyframeSet new_kfs;
	for(int i = 0; i < msg->kf_num; i++)
	{
		Keyframe kf;

		for(int j = 0; j < 3; j++)
		{
			kf.position[j] = msg->kfs[i].position[j];
			kf.velocity[j] = msg->kfs[i].velocity[j];
		}
		kf.vel_constr = msg->kfs[i].vel_constr;
		kf.yaw = msg->kfs[i].yaw;

		new_kfs.keyframes.push_back(kf);
	}
	new_kfs.start_time = msg->sys_time.time_stamp;

	GenerateTrajectory(new_kfs, msg->path_id);
}

double PathManager::CalcFlightTime(Position3Dd start, Position3Dd goal, double vel)
{
	double xe = start.x - goal.x;
	double ye = start.y - goal.y;
	double ze = start.z - goal.z;

	double dist = std::sqrt(std::pow(xe, 2) + std::pow(ye, 2) + std::pow(ze, 2));
	std::cout << "dist: " << dist << ", allocated time: " << dist/vel << std::endl;

	return dist/vel;
}

void PathManager::GenerateTrajectory(KeyframeSet& kfs, uint64_t traj_id)
{
	srcl_lcm_msgs::PolynomialCurve_t poly_msg;
	uint8_t kf_num = kfs.keyframes.size();

	if(kf_num < 2)
		return;

	poly_msg.wp_num = kfs.keyframes.size();

	//	traj_opt_.InitOptJointMatrices(kf_num);
	QuadPolyOpt traj_opt_;
	traj_opt_.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);

	for(int i = 0; i < kfs.keyframes.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0,i) = kfs.keyframes[i].position[0];
		traj_opt_.keyframe_y_vals_(0,i) = kfs.keyframes[i].position[1];
		traj_opt_.keyframe_z_vals_(0,i) = kfs.keyframes[i].position[2];

		if(kfs.keyframes[i].vel_constr)
		{
			traj_opt_.keyframe_x_vals_(1,i) = kfs.keyframes[i].velocity[0];
			traj_opt_.keyframe_y_vals_(1,i) = kfs.keyframes[i].velocity[1];
			traj_opt_.keyframe_z_vals_(1,i) = kfs.keyframes[i].velocity[2];
		}
		else
		{
			traj_opt_.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();
		}

		traj_opt_.keyframe_x_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(2,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(3,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_yaw_vals_(0,i) = kfs.keyframes[i].yaw;
		traj_opt_.keyframe_yaw_vals_(1,i) = std::numeric_limits<float>::infinity();

		//traj_opt_.keyframe_ts_(0,i) = i * 1.0;

		if(i == 0) {
			traj_opt_.keyframe_ts_(0,i) = 0;
		}
		else {
			traj_opt_.keyframe_ts_(0,i) = traj_opt_.keyframe_ts_(0,i-1) + 1.0;

//			Position3Dd start(traj_opt_.keyframe_x_vals_(0,i-1), traj_opt_.keyframe_y_vals_(0,i-1), traj_opt_.keyframe_z_vals_(0,i-1));
//			Position3Dd goal(traj_opt_.keyframe_x_vals_(0,i), traj_opt_.keyframe_y_vals_(0,i), traj_opt_.keyframe_z_vals_(0,i));
//
//			double tp = CalcFlightTime(start, goal, 2.0);
//			traj_opt_.keyframe_ts_(0,i) = traj_opt_.keyframe_ts_(0,i-1) + tp*5;
		}
//		std::cout << "allocated time: " << traj_opt_.keyframe_ts_(0,i) << std::endl;

		srcl_lcm_msgs::WayPoint_t wpoint;
		wpoint.positions[0] = kfs.keyframes[i].position[0];
		wpoint.positions[1] = kfs.keyframes[i].position[1];
		wpoint.positions[2] = kfs.keyframes[i].position[2];
		poly_msg.waypoints.push_back(wpoint);
	}

	traj_opt_.keyframe_x_vals_(1,0) = 0.0;
	traj_opt_.keyframe_y_vals_(1,0) = 0.0;
	traj_opt_.keyframe_z_vals_(1,0) = 0.0;

	traj_opt_.keyframe_x_vals_(2,0) = 0.0;
	traj_opt_.keyframe_y_vals_(2,0) = 0.0;
	traj_opt_.keyframe_z_vals_(2,0) = 0.0;

	traj_opt_.keyframe_x_vals_(3,0) = 0.0;
	traj_opt_.keyframe_y_vals_(3,0) = 0.0;
	traj_opt_.keyframe_z_vals_(3,0) = 0.0;

	traj_opt_.keyframe_yaw_vals_(1,0) = 0;

	traj_opt_.keyframe_x_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(2,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_y_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_z_vals_(3,kf_num - 1) = 0;

	traj_opt_.keyframe_yaw_vals_(1,kf_num - 1) = 0;

	//traj_opt_.OptimizeFlatTrajJoint();
	bool result = traj_opt_.OptimizeFlatTrajWithCorridorJoint();

	// send results to LCM network
	if(result)
	{
		poly_msg.seg_num = traj_opt_.flat_traj_.traj_segs_.size();
		for(auto& seg : traj_opt_.flat_traj_.traj_segs_)
		{
			srcl_lcm_msgs::PolyCurveSegment_t seg_msg;

			seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
			seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
			seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
			seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
			for(auto& coeff:seg.seg_x.param_.coeffs)
				seg_msg.coeffs_x.push_back(coeff);
			for(auto& coeff:seg.seg_y.param_.coeffs)
				seg_msg.coeffs_y.push_back(coeff);
			for(auto& coeff:seg.seg_z.param_.coeffs)
				seg_msg.coeffs_z.push_back(coeff);
			for(auto& coeff:seg.seg_yaw.param_.coeffs)
				seg_msg.coeffs_yaw.push_back(coeff);

			seg_msg.t_start = seg.t_start;
			seg_msg.t_end = seg.t_end;

			poly_msg.segments.push_back(seg_msg);
		}
		poly_msg.start_time.time_stamp = kfs.start_time;
		poly_msg.trajectory_id = traj_id;

		poly_msg.scaling_factor = 0.3;

		lcm_->publish("quad_planner/trajectory_polynomial", &poly_msg);
	}
}


