/*
 * test_combiner.cpp
 *
 *  Created on: Sep 12, 2016
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
	double desired_height = 0.8;

	// read octomap data
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.35);
	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/build/bin/octree_obstacle_test_36.bt";
	//"/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set3/octree_from_server_node_eset3.bt";
	tree->readBinary(tree_path);

	// read 2d map data
	Mat input_image;
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set3/map_path_repair.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
//	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMapWithExtObstacle(input_image, 32,1);
	sgrid_map.info.SetWorldSize(5.0, 5.0);
	sgrid_map.info.origin_offset_x = 2.5;
	sgrid_map.info.origin_offset_y = 2.5;

	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	//std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctreeWithExtObstacle(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	////////////////////////////////////////////////////////////////////////////////////////////

	// combine graphs
	GraphCombiner<SquareCell*, SquareGrid> combiner;
	combiner.UpdateVehiclePose(Position3Dd(-1.8,0.6,0.8), Eigen::Quaterniond(1 , 0 , 0 , 0));
	combiner.SetBaseGraph(map_graph, sgrid_map.data_model, sgrid_map.data_model->cells_.size(), sgrid_map.info);
	combiner.SetDesiredHeight(desired_height);

	uint64_t geo_start_id_astar;

	clock_t		exec_time;
	exec_time = clock();
	geo_start_id_astar = combiner.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
	exec_time = clock() - exec_time;
	std::cout << "Combined graph construction finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::cout << "2D graph size: " << map_graph->GetGraphVertices().size() <<
			", combined graph size: " << combiner.combined_graph_.GetGraphVertices().size() << std::endl;

	if(geo_start_id_astar == -1)
		return 1;

	// search in combined graph
	//uint64_t geo_start_id_astar = map_graph->GetVertexFromID(844)->bundled_data_->geo_mark_id_;
	uint64_t geo_goal_id_astar = map_graph->GetVertexFromID(187)->bundled_data_->geo_mark_id_;

	exec_time = clock();
	auto comb_path = AStar::Search(combiner.combined_graph_, geo_start_id_astar, geo_goal_id_astar);
	exec_time = clock() - exec_time;
	std::cout << "Search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::vector<Position3Dd> comb_path_pos;
	for(auto& wp:comb_path)
		comb_path_pos.push_back(wp->bundled_data_.position);

	double est_new_dist = 0;
	for(int i = 0; i < comb_path.size() - 1; i++)
		est_new_dist += std::sqrt(std::pow(comb_path_pos[i].x - comb_path_pos[i + 1].x,2) +
				std::pow(comb_path_pos[i].y - comb_path_pos[i + 1].y,2) +
				std::pow(comb_path_pos[i].z - comb_path_pos[i + 1].z,2));
	std::cout << "total cost of path: " << est_new_dist << std::endl;

//	std::vector<Position3Dd> octomap_waypoints;
//	uint32_t select_num = 0;
//	octomap_waypoints.push_back(comb_path_pos[0]);
//	for(auto& wp:comb_path)
//	{
//		if(wp->bundled_data_.source == GeoMarkSource::LASER_OCTOMAP) {
//			octomap_waypoints.push_back(wp->bundled_data_.position);
//
////			if(select_num++ == 2)
////				break;
//		}
//	}
//
//	uint8_t kf_num = octomap_waypoints.size();
//
//	QuadPolyOpt opt;
//	if(kf_num < 2)
//		return -1;
//
//	//opt.InitOptJointMatrices(kf_num);
//	opt.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);
//
//	for(int i = 0; i < octomap_waypoints.size(); i++)
//	{
//		opt.keyframe_x_vals_(0,i) = octomap_waypoints[i].x;
//		opt.keyframe_y_vals_(0,i) = octomap_waypoints[i].y;
//		opt.keyframe_z_vals_(0,i) = octomap_waypoints[i].z;
//		opt.keyframe_yaw_vals_(0,i) = 0;
//
//		opt.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
//		opt.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
//		opt.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();
//
//		opt.keyframe_ts_(0,i) = i * 1.0;
//	}
//
//	opt.keyframe_x_vals_(1,0) = 0.0;
//	opt.keyframe_y_vals_(1,0) = 0.0;
//	opt.keyframe_z_vals_(1,0) = 0.0;
//
//	opt.keyframe_x_vals_(1,kf_num - 1) = 0;
//	opt.keyframe_y_vals_(1,kf_num - 1) = 0;
//	opt.keyframe_z_vals_(1,kf_num - 1) = 0;
//
//	//opt.OptimizeFlatTrajJoint();
//	opt.OptimizeFlatTrajWithCorridorJoint();
	//std::cout << "octomap_waypoints length: " << octomap_waypoints.size() << std::endl;

	////////////////////////////////////////////////////////////////////////////////////////////
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_lcm_msgs::Graph_t graph_msg3;

	graph_msg3.vertex_num = combiner.combined_graph_.GetGraphVertices().size();
	for(auto& vtx : combiner.combined_graph_.GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.position.x;
		vertex.position[1] = vtx->bundled_data_.position.y;
		vertex.position[2] = vtx->bundled_data_.position.z;

		graph_msg3.vertices.push_back(vertex);
	}

	graph_msg3.edge_num = combiner.combined_graph_.GetGraphUndirectedEdges().size();
	for(auto& eg : combiner.combined_graph_.GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg3.edges.push_back(edge);
	}

//	graph_msg3.vertex_num = cubegraph->GetGraphVertices().size();
//	for(auto& vtx : cubegraph->GetGraphVertices())
//	{
//		srcl_lcm_msgs::Vertex_t vertex;
//		vertex.id = vtx->vertex_id_;
//
//		vertex.position[0] = vtx->bundled_data_.location_.x;
//		vertex.position[1] = vtx->bundled_data_.location_.y;
//		vertex.position[2] = vtx->bundled_data_.location_.z;
//
//		graph_msg3.vertices.push_back(vertex);
//	}
//
//	graph_msg3.edge_num = cubegraph->GetGraphUndirectedEdges().size();
//	for(auto& eg : cubegraph->GetGraphUndirectedEdges())
//	{
//		srcl_lcm_msgs::Edge_t edge;
//		edge.id_start = eg.src_->vertex_id_;
//		edge.id_end = eg.dst_->vertex_id_;
//
//		graph_msg3.edges.push_back(edge);
//	}

	lcm->publish("quad_planner/cube_graph", &graph_msg3);

	srcl_lcm_msgs::Path_t path_msg;

	path_msg.waypoint_num = comb_path.size();
	for(auto& wp : comb_path_pos)
	{
		srcl_lcm_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}

	lcm->publish("quad_planner/geo_mark_graph_path", &path_msg);

//	srcl_lcm_msgs::PolynomialCurve_t poly_msg;
//
//	poly_msg.seg_num = opt.flat_traj_.traj_segs_.size();
//
//	std::cout << "seg num: " << poly_msg.seg_num << std::endl;
//	for(auto& seg : opt.flat_traj_.traj_segs_)
//	{
//		srcl_lcm_msgs::PolyCurveSegment_t seg_msg;
//
//		seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
//		seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
//		seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
//		seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
//
//		for(auto& coeff:seg.seg_x.param_.coeffs)
//			seg_msg.coeffs_x.push_back(coeff);
//		for(auto& coeff:seg.seg_y.param_.coeffs)
//			seg_msg.coeffs_y.push_back(coeff);
//		for(auto& coeff:seg.seg_z.param_.coeffs)
//			seg_msg.coeffs_z.push_back(coeff);
//		for(auto& coeff:seg.seg_yaw.param_.coeffs)
//			seg_msg.coeffs_yaw.push_back(coeff);
//
//		seg_msg.t_start = 0;
//		seg_msg.t_end = 1.0;
//
//		poly_msg.segments.push_back(seg_msg);
//	}
//
//	lcm->publish("quad_planner/polynomial_curve", &poly_msg);

	return 0;
}
