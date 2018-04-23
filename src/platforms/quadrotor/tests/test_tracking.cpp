/*
 * test_tracking.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: rdu
 */

#include <memory>
#include <ctime>
#include <cmath>

#include "eigen3/Eigen/Core"

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

// opencv
#include "opencv2/opencv.hpp"

#include "path_repair/graph_combiner.h"
#include "geometry/square_grid/square_grid.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/cube_array_builder.h"
#include "map/map_utils.h"
#include "quad/quad_polyopt.h"

using namespace librav;
using namespace octomap;
using namespace cv;
using namespace Eigen;

int main(int argc, char* argv[])
{
	std::vector<Position3Dd> waypoints;

	waypoints.push_back(Position3Dd(0.0, 0.0, 0.5));
	waypoints.push_back(Position3Dd(0.8, 0.8, 0.6));
	waypoints.push_back(Position3Dd(2.0, 1.2, 0.7));
	waypoints.push_back(Position3Dd(2.8, 2.0, 0.8));
//	waypoints.push_back(Position3Dd(0.0, 0.0, 0.5));
//	waypoints.push_back(Position3Dd(1.6, 1.6, 0.6));
//	waypoints.push_back(Position3Dd(4.0, 2.5, 0.7));
//	waypoints.push_back(Position3Dd(6.0, 5.0, 0.8));

	uint8_t kf_num = waypoints.size();

	QuadPolyOpt opt;
	if(kf_num < 2)
		return -1;

	//opt.InitOptJointMatrices(kf_num);
	opt.SetPositionPolynomialOrder(10);
	opt.InitOptWithCorridorJointMatrices(kf_num, 20, 0.05);

	for(int i = 0; i < waypoints.size(); i++)
	{
		opt.keyframe_x_vals_(0,i) = waypoints[i].x;
		opt.keyframe_y_vals_(0,i) = waypoints[i].y;
		opt.keyframe_z_vals_(0,i) = waypoints[i].z;

		opt.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();

		opt.keyframe_x_vals_(2,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_y_vals_(2,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_z_vals_(2,i) = std::numeric_limits<float>::infinity();

		opt.keyframe_x_vals_(3,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_y_vals_(3,i) = std::numeric_limits<float>::infinity();
		opt.keyframe_z_vals_(3,i) = std::numeric_limits<float>::infinity();

//		opt.keyframe_yaw_vals_(0,i) = std::numeric_limits<float>::infinity();
//		opt.keyframe_yaw_vals_(1,i) = std::numeric_limits<float>::infinity();

		opt.keyframe_ts_(0,i) = i * 0.5;
	}

	opt.keyframe_x_vals_(1,0) = 0.0;
	opt.keyframe_y_vals_(1,0) = 0.0;
	opt.keyframe_z_vals_(1,0) = 0.0;

	opt.keyframe_x_vals_(2,0) = 0.0;
	opt.keyframe_y_vals_(2,0) = 0.0;
	opt.keyframe_z_vals_(2,0) = 0.0;

	opt.keyframe_x_vals_(3,0) = 0.0;
	opt.keyframe_y_vals_(3,0) = 0.0;
	opt.keyframe_z_vals_(3,0) = 0.0;

	opt.keyframe_x_vals_(1,kf_num - 1) = 0;
	opt.keyframe_y_vals_(1,kf_num - 1) = 0;
	opt.keyframe_z_vals_(1,kf_num - 1) = 0;

	opt.keyframe_x_vals_(2,kf_num - 1) = 0;
	opt.keyframe_y_vals_(2,kf_num - 1) = 0;
	opt.keyframe_z_vals_(2,kf_num - 1) = 0;

	opt.keyframe_x_vals_(3,kf_num - 1) = 0;
	opt.keyframe_y_vals_(3,kf_num - 1) = 0;
	opt.keyframe_z_vals_(3,kf_num - 1) = 0;

//	opt.keyframe_yaw_vals_(0,0) = 0;
//	opt.keyframe_yaw_vals_(0,kf_num - 1) = M_PI/4.0;
//
//	opt.keyframe_yaw_vals_(1,0) = 0;
//	opt.keyframe_yaw_vals_(1,kf_num - 1) = 0;

	//opt.OptimizeFlatTrajJoint();
	opt.OptimizeFlatTrajWithCorridorJoint();

	////////////////////////////////////////////////////////////////////////////////////////////
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_lcm_msgs::Path_t path_msg;

	path_msg.waypoint_num = waypoints.size();
	for(auto& wp : waypoints)
	{
		srcl_lcm_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}

	lcm->publish("quad_planner/geo_mark_graph_path", &path_msg);

	srcl_lcm_msgs::PolynomialCurve_t poly_msg;

	poly_msg.seg_num = opt.flat_traj_.traj_segs_.size();
	for(auto& seg : opt.flat_traj_.traj_segs_)
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

	lcm->publish("quad_planner/trajectory_polynomial", &poly_msg);

	return 0;
}


