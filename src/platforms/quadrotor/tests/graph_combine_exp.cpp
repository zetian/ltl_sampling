/*
 * graph_combiner.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#include <memory>
#include <iostream>
#include <ctime>
#include <cmath>
#include <set>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/map_type.h"
#include "map/map_utils.h"
#include "planning/graph/graph.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "vis/graph_vis.h"
#include "path_repair/geo_mark.h"

#include "common/librav_math.hpp"

using namespace librav;
using namespace octomap;
using namespace cv;

int main(int argc, char* argv[])
{
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);

	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set2/octree_set2.bt";
	tree->readBinary(tree_path);

	Mat input_image;
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set1/map_path_repair.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	sgrid_map.info.SetWorldSize(5.0, 5.0);
	sgrid_map.info.origin_offset_x = 2.5;
	sgrid_map.info.origin_offset_y = 2.5;

	std::cout << "\n*********************************************************\n" << std::endl;

	std::cout << "num of leaf nodes: " << tree->getNumLeafNodes() << std::endl;
	std::cout << "tree depth: " <<  tree->getTreeDepth() << std::endl;

	std::cout << "\n---------------------------------------------------------\n" << std::endl;

	clock_t		exec_time;
	exec_time = clock();

	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	Graph_t<GeoMark> comb_graph;

	GeoMark mark1, mark2;
	uint64_t comb_graph_idx = 0;

	utils::Transformation::Transform3D transf;
	transf.trans = utils::Transformation::Translation3D(-1.8,0.6,0.8);
	transf.quat = Eigen::Quaterniond(0.923868 , 3.68874e-05 , 9.55242e-06 , -0.382712);

	for(auto& edge:cubegraph->GetGraphEdges())
	{
		mark1.data_id_ = edge.src_->vertex_id_;
		mark1.position = utils::Transformation::TransformPosition3D(transf, edge.src_->bundled_data_.location_);
		mark1.source = GeoMarkSource::LASER_OCTOMAP;
		mark1.source_id = edge.src_->bundled_data_.data_id_;
		edge.src_->bundled_data_.geo_mark_id_ = mark1.data_id_;

		mark2.data_id_ = edge.dst_->vertex_id_;
		mark2.position =  utils::Transformation::TransformPosition3D(transf, edge.dst_->bundled_data_.location_);
		mark2.source = GeoMarkSource::LASER_OCTOMAP;
		mark2.source_id = edge.dst_->bundled_data_.data_id_;
		edge.dst_->bundled_data_.geo_mark_id_ = mark2.data_id_;

		comb_graph.AddEdge(mark1, mark2, edge.cost_);
	}

	uint64_t existing_node_num = sgrid_map.data_model->cells_.size();
	for(auto& edge:map_graph->GetGraphEdges())
	{
		mark1.data_id_ = existing_node_num + edge.src_->vertex_id_;
		Position2Dd ref_world_pos1 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.src_->bundled_data_->location_, sgrid_map.info);
		mark1.position.x = ref_world_pos1.x;
		mark1.position.y = ref_world_pos1.y;
		mark1.position.z = 0.8;
		mark1.source = GeoMarkSource::PLANAR_MAP;
		mark1.source_id = edge.src_->bundled_data_->data_id_;
		edge.src_->bundled_data_->geo_mark_id_ = mark1.data_id_;

		mark2.data_id_ = existing_node_num + edge.dst_->vertex_id_;
		Position2Dd ref_world_pos2 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.dst_->bundled_data_->location_, sgrid_map.info);
		mark2.position.x = ref_world_pos2.x;
		mark2.position.y = ref_world_pos2.y;
		mark2.position.z = 0.8;
		mark2.source = GeoMarkSource::PLANAR_MAP;
		mark2.source_id = edge.dst_->bundled_data_->data_id_;
		edge.dst_->bundled_data_->geo_mark_id_ = mark2.data_id_;

		comb_graph.AddEdge(mark1, mark2, edge.cost_);
	}

	// connect starting points
	GeoMark map2d_start_mark, cube_start_mark;

	Position2Di map2d_start_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(-1.8,0.6), sgrid_map.info);
	uint64_t map2d_start_id = sgrid_map.data_model->GetIDFromPosition(map2d_start_pos.x, map2d_start_pos.y);
	uint64_t geo_start_id = map_graph->GetVertexFromID(map2d_start_id)->bundled_data_->geo_mark_id_;
	map2d_start_mark = comb_graph.GetVertexFromID(geo_start_id)->bundled_data_;

	std::set<uint32_t> hei_set;
	for(auto& st_cube : cubearray->GetStartingCubes())
	{
		uint64_t cube_start_id = cubearray->cubes_[st_cube].geo_mark_id_;
		cube_start_mark = comb_graph.GetVertexFromID(cube_start_id)->bundled_data_;

		double cost = std::sqrt(std::pow(map2d_start_mark.position.x - cube_start_mark.position.x, 2) +
				std::pow(map2d_start_mark.position.y - cube_start_mark.position.y, 2) +
				std::pow(map2d_start_mark.position.z - cube_start_mark.position.z, 2));
		comb_graph.AddEdge(map2d_start_mark, cube_start_mark, cost);

		hei_set.insert(cubearray->cubes_[st_cube].index_.z);
	}

	// get all vertices of the cube graph around the current flight height
	std::vector<uint64_t> hei_vertices;
	for(auto& vtx:cubegraph->GetGraphVertices())
	{
		for(auto& hei_ele:hei_set)
		{
			if(vtx->bundled_data_.index_.z == hei_ele)
			{
				hei_vertices.push_back(vtx->bundled_data_.geo_mark_id_);
				break;
			}
		}
	}

	for(auto& v:hei_vertices)
	{
		GeoMark mark2d, mark3d;
		mark3d = comb_graph.GetVertexFromID(v)->bundled_data_;

		Position2Di map_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(mark3d.position.x,mark3d.position.y), sgrid_map.info);
		uint64_t map2d_id = sgrid_map.data_model->GetIDFromPosition(map_pos.x, map_pos.y);
		if(map_graph->GetVertexFromID(map2d_id) == nullptr)
			continue;
		uint64_t geo2d_id = map_graph->GetVertexFromID(map2d_id)->bundled_data_->geo_mark_id_;
		mark2d = comb_graph.GetVertexFromID(geo2d_id)->bundled_data_;

		double cost = std::sqrt(std::pow(mark3d.position.x - mark2d.position.x, 2) +
				std::pow(mark3d.position.y - mark2d.position.y, 2) +
				std::pow(mark3d.position.z - mark2d.position.z, 2));
		comb_graph.AddEdge(mark3d, mark2d, cost);
	}

	exec_time = clock() - exec_time;
	std::cout << "Graph construction finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

//	Position2Di geo_goal_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(1.8, -2.0), sgrid_map.info);
//	uint64_t map_goal_id = sgrid_map.data_model->GetIDFromPosition(geo_goal_pos.x, geo_goal_pos.y);
//	uint64_t geo_goal_id = map_graph->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;

	uint64_t geo_start_id_astar = map_graph->GetVertexFromID(844)->bundled_data_->geo_mark_id_;
	uint64_t geo_goal_id_astar = map_graph->GetVertexFromID(187)->bundled_data_->geo_mark_id_;

//	std::cout << "start id: " << map2d_start_id << " , " << geo_start_id << std::endl;
//	std::cout << "goal id: " << map_goal_id << " , " << geo_goal_id << std::endl;
	auto comb_path = comb_graph.AStarSearch(geo_start_id_astar, geo_goal_id_astar);
	std::vector<Position3Dd> comb_path_pos;
	for(auto& wp:comb_path)
		comb_path_pos.push_back(wp->bundled_data_.position);

	/*-------------------------------------------------------------------------------------*/
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
	for(auto& vtx : cubegraph->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

//		vertex.position[0] = vtx->bundled_data_.location_.x;
//		vertex.position[1] = vtx->bundled_data_.location_.y;
//		vertex.position[2] = vtx->bundled_data_.location_.z;

		utils::Transformation::Transform3D transf;
		transf.trans = utils::Transformation::Translation3D(-1.8,0.6,0.8+0.11);
		//Eigen::Quaterniond rot(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
		transf.quat = Eigen::Quaterniond(0.923868 , 3.68874e-05 , 9.55242e-06 , -0.382712);
		Position3Dd pos_world = utils::Transformation::TransformPosition3D(transf, vtx->bundled_data_.location_);
		vertex.position[0] = pos_world.x;
		vertex.position[1] = pos_world.y;
		vertex.position[2] = pos_world.z;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
	for(auto& eg : cubegraph->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	//lcm->publish("quad_planner/cube_graph", &graph_msg);

	srcl_msgs::Graph_t graph_msg2;

	graph_msg2.vertex_num = map_graph->GetGraphVertices().size();
	for(auto& vtx : map_graph->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(vtx->bundled_data_->location_, sgrid_map.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg2.vertices.push_back(vertex);
	}

	graph_msg2.edge_num = map_graph->GetGraphUndirectedEdges().size();
	for(auto& eg : map_graph->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg2.edges.push_back(edge);
	}

	//lcm->publish("quad_planner/quad_planner_graph", &graph_msg2);

	//////////////////////////////////////////////////////
	srcl_msgs::Path_t path_msg;

	path_msg.waypoint_num = comb_path.size();
	for(auto& wp : comb_path_pos)
	{
		srcl_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}
	lcm->publish("quad_planner/geo_mark_graph_path", &path_msg);

	srcl_msgs::Graph_t graph_msg3;

	graph_msg3.vertex_num = comb_graph.GetGraphVertices().size();
	for(auto& vtx : comb_graph.GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.position.x;
		vertex.position[1] = vtx->bundled_data_.position.y;
		vertex.position[2] = vtx->bundled_data_.position.z;

		graph_msg3.vertices.push_back(vertex);
	}

	graph_msg3.edge_num = comb_graph.GetGraphUndirectedEdges().size();
	for(auto& eg : comb_graph.GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg3.edges.push_back(edge);
	}

	lcm->publish("quad_planner/cube_graph", &graph_msg3);
}


