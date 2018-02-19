/*
 * test_cubearray_builder.cpp
 *
 *  Created on: Nov 17, 2016
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
//	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set3/octree_from_server_node_eset3.bt";
//	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/octomap/octree_obstacle_test.bt";
	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/librav/build/bin/octree_obstacle_test_36.bt";
	tree->readBinary(tree_path);

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctreeWithExtObstacle(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_lcm_msgs::Graph_t graph_msg3;

	graph_msg3.vertex_num = cubegraph->GetGraphVertices().size();
	for(auto& vtx : cubegraph->GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.location_.x;
		vertex.position[1] = vtx->bundled_data_.location_.y;
		vertex.position[2] = vtx->bundled_data_.location_.z;

		graph_msg3.vertices.push_back(vertex);
	}

	graph_msg3.edge_num = cubegraph->GetGraphUndirectedEdges().size();
	for(auto& eg : cubegraph->GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg3.edges.push_back(edge);
	}

	lcm->publish("quad_planner/cube_graph", &graph_msg3);

	return 0;
}


