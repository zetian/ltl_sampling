/*
 * test_combiner.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: rdu
 */

#include <geo_mark_graph.h>
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
	//std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/geomark/octree_obstacle_test_36.bt";
	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/geomark/case4.bt";
	tree->readBinary(tree_path);

	// read 2d map data
	Mat input_image;
	//std::string image_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set3/map_path_repair.png";
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_testcase4.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
	Map_t<SquareGrid> sgrid_map = SGridBuilderV2::BuildSquareGridMap(input_image, 32);
//	sgrid_map.info.SetWorldSize(5.0, 5.0);
//	sgrid_map.info.origin_offset_x = 2.5;
//	sgrid_map.info.origin_offset_y = 2.5;
	sgrid_map.info.SetWorldSize(10.0, 18.0);
	sgrid_map.info.origin_offset_x = 5.0;
	sgrid_map.info.origin_offset_y = 9.0;

	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	//std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctreeWithExtObstacle(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	std::cout << "cube array info: \nSize(r,c,h): " << cubearray->row_size_ << " , " << cubearray->col_size_ << " , " << cubearray->hei_size_
			<< " ; Offset: " << cubearray->row_offset_ << " , " << cubearray->col_offset_ << " , " << cubearray->hei_offset_ << std::endl;

	//cubearray->GetFreeCubesAroundHeight(0.8);

	////////////////////////////////////////////////////////////////////////////////////////////

	// combine graphs
	GeoMarkGraph combiner;
	// -1.65,0.8,0.8,-M_PI/5
	Eigen::Quaterniond quat(Eigen::AngleAxisd(-M_PI/5, Eigen::Vector3d::UnitZ()));
//	combiner.UpdateVehiclePose(Position3Dd(-1.65,0.8,0.8), quat);//Eigen::Quaterniond(1 , 0 , 0 , 0));
	combiner.UpdateVehiclePose(Position3Dd(-1.35,-0.5,1.5), quat);//Eigen::Quaterniond(1 , 0 , 0 , 0));
//	combiner.UpdateSquareGridInfo(map_graph, sgrid_map.data_model, sgrid_map.data_model->cells_.size(), sgrid_map.info);
	combiner.UpdateSquareGridInfo(map_graph, sgrid_map);
	combiner.SetGoalHeightRange(0.5, 1.5);

	int64_t geo_start_id_astar;

	clock_t		exec_time;
	exec_time = clock();
	geo_start_id_astar = combiner.MergeCubeArrayInfo(cubegraph, cubearray);
	exec_time = clock() - exec_time;
	std::cout << "Combined graph construction finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::cout << "2D graph size: " << map_graph->GetGraphVertices().size() <<
			", combined graph size: " << combiner.combined_graph_.GetGraphVertices().size() << std::endl;

	if(geo_start_id_astar == -1)
		return 1;

	// search in combined graph
//	uint64_t geo_start_id_astar = map_graph->GetVertexFromID(844)->bundled_data_->geo_mark_id_;
//	uint64_t geo_goal_id_astar = map_graph->GetVertexFromID(26)->bundled_data_->geo_mark_id_;
//
//	exec_time = clock();
//	auto comb_path = AStar::Search(combiner.combined_graph_, geo_start_id_astar, geo_goal_id_astar);
//	exec_time = clock() - exec_time;
//	std::cout << "Search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
//
//	std::vector<Position3Dd> comb_path_pos;
//	for(auto& wp:comb_path)
//		comb_path_pos.push_back(wp->bundled_data_.position);
//
//	double est_new_dist = 0;
//	for(int i = 0; i < comb_path.size() - 1; i++)
//		est_new_dist += std::sqrt(std::pow(comb_path_pos[i].x - comb_path_pos[i + 1].x,2) +
//				std::pow(comb_path_pos[i].y - comb_path_pos[i + 1].y,2) +
//				std::pow(comb_path_pos[i].z - comb_path_pos[i + 1].z,2));
//	std::cout << "total cost of path: " << est_new_dist << std::endl;

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

	lcm->publish("quad_planner/geo_mark_graph", &graph_msg3);

//	if(!comb_path.empty()) {
//		srcl_lcm_msgs::Path_t path_msg;
//
//		path_msg.waypoint_num = comb_path.size();
//		for(auto& wp : comb_path_pos)
//		{
//			srcl_lcm_msgs::WayPoint_t waypoint;
//			waypoint.positions[0] = wp.x;
//			waypoint.positions[1] = wp.y;
//			waypoint.positions[2] = wp.z;
//
//			path_msg.waypoints.push_back(waypoint);
//		}
//
//		lcm->publish("quad_planner/geo_mark_graph_path", &path_msg);
//	}

	std::cout << "----" << std::endl;

	return 0;
}
