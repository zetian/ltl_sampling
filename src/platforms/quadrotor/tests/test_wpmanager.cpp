/*
 * test_wpmanager.cpp
 *
 *  Created on: Oct 31, 2016
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
#include <path_manager.h>
#include "lcmtypes/librav.hpp"

// opencv
#include "opencv2/opencv.hpp"

#include "planning/graph/astar.h"
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
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	// read octomap data
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);
	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set2/octree_set2.bt";
	tree->readBinary(tree_path);

	// read 2d map data
	Mat input_image;
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set1/map_path_repair.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	sgrid_map.info.SetWorldSize(5.0, 5.0);
	sgrid_map.info.origin_offset_x = 2.5;
	sgrid_map.info.origin_offset_y = 2.5;

	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	////////////////////////////////////////////////////////////////////////////////////////////

	// combine graphs
	GraphCombiner<SquareCell*, SquareGrid> combiner;
	combiner.UpdateVehiclePose(Position3Dd(-1.8,0.6,0.8), Eigen::Quaterniond(0.923868 , 3.68874e-05 , 9.55242e-06 , -0.382712));
	combiner.SetBaseGraph(map_graph, sgrid_map.data_model, sgrid_map.data_model->cells_.size(), sgrid_map.info);

	clock_t		exec_time;
	exec_time = clock();
	combiner.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
	exec_time = clock() - exec_time;
	std::cout << "Graph construction finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	exec_time = clock();
	combiner.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
	exec_time = clock() - exec_time;
	std::cout << "Graph construction 2 finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	// search in combined graph
	uint64_t geo_start_id_astar = map_graph->GetVertexFromID(844)->bundled_data_->geo_mark_id_;
	uint64_t geo_goal_id_astar = map_graph->GetVertexFromID(187)->bundled_data_->geo_mark_id_;

	exec_time = clock();
	auto comb_path = AStar::Search(combiner.combined_graph_, geo_start_id_astar, geo_goal_id_astar);
	exec_time = clock() - exec_time;
	std::cout << "Search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::vector<Position3Dd> comb_path_pos;
	for(auto& wp:comb_path)
		comb_path_pos.push_back(wp->bundled_data_.position);
	std::vector<Position3Dd> selected_wps = MissionUtils::GetKeyTurningWaypoints(comb_path_pos);

	////////////////////////////////////////////////////////////////////////////////////////////
	// send data for visualization
//	srcl_lcm_msgs::Path_t path_msg;
//
//	path_msg.waypoint_num = comb_path.size();
//	for(auto& wp : comb_path_pos)
//	{
//		srcl_lcm_msgs::WayPoint_t waypoint;
//		waypoint.positions[0] = wp.x;
//		waypoint.positions[1] = wp.y;
//		waypoint.positions[2] = wp.z;
//
//		path_msg.waypoints.push_back(waypoint);
//	}
//
//	lcm->publish("quad_planner/geo_mark_graph_path", &path_msg);
	srcl_lcm_msgs::KeyframeSet_t kf_cmd;

	Eigen::Vector3d goal_vec(selected_wps.back().x, selected_wps.back().y, 0);

	kf_cmd.kf_num = selected_wps.size();
	for(auto& wp:selected_wps)
	{
		srcl_lcm_msgs::Keyframe_t kf;
		kf.vel_constr = false;

		kf.positions[0] = wp.x;
		kf.positions[1] = wp.y;
		kf.positions[2] = wp.z;

		Eigen::Vector3d pos_vec(wp.x, wp.y, 0);
		Eigen::Vector3d dir_vec = goal_vec - pos_vec;
		Eigen::Vector3d x_vec(1,0,0);
		double angle = - std::acos(dir_vec.normalized().dot(x_vec));
		kf.yaw = angle;

		kf_cmd.kfs.push_back(kf);
	}
	kf_cmd.kfs.front().yaw = 0;
	kf_cmd.kfs.back().yaw = -M_PI/4;

	lcm->publish("quad_planner/goal_keyframe_set", &kf_cmd);

	return 0;
}
