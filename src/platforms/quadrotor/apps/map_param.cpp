/*
 * map_param.cpp
 *
 *  Created on: Nov 21, 2016
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// opencv
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

// user
#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"
#include "map/image_utils.h"
#include "map/map_utils.h"
#include "path_repair/graph_planner.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"

using namespace cv;
using namespace librav;

template<typename PlannerType>
srcl_lcm_msgs::Graph_t GetLcmGraphFromPlanner(const PlannerType& planner)
{
	srcl_lcm_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = planner.graph_->GetGraphVertices().size();
	for(auto& vtx : planner.graph_->GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(vtx->bundled_data_->location_, planner.map_.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = planner.graph_->GetGraphUndirectedEdges().size();
	for(auto& eg : planner.graph_->GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	return graph_msg;
}

int main(int argc, char** argv )
{
	bool show_padding = false;
	GraphPlanner<SquareGrid> sgrid_planner;

	/*** Config graph planner ***/
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/map_testcase2.png";
	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	map_config.SetOriginOffset(10.0, 12.5);

	sgrid_planner.UpdateMapConfig(map_config);
	sgrid_planner.map_.info.SetWorldSize(20.0, 25.0);

	/*** Search path in the graph ***/
	Position2Dd start_w(-11.0,8.5);
	Position2Dd goal_w(11.0, -8.5);

	Position2Di start_m = MapUtils::CoordinatesFromRefWorldToMapPadded(start_w, sgrid_planner.map_.info);
	Position2Di goal_m = MapUtils::CoordinatesFromRefWorldToMapPadded(goal_w, sgrid_planner.map_.info);

	auto start_vertex_id = sgrid_planner.map_.data_model->GetIDFromPosition(start_m.x, start_m.y);
	auto goal_vertex_id = sgrid_planner.map_.data_model->GetIDFromPosition(goal_m.x, goal_m.y);

	auto start_vertex = sgrid_planner.graph_->GetVertexFromID(start_vertex_id);
	auto goal_vertex = sgrid_planner.graph_->GetVertexFromID(goal_vertex_id);

	Path_t<SquareCell*> path;
	if(start_vertex == nullptr || goal_vertex == nullptr)
	{
		if(start_vertex == nullptr)
			std::cout << "Invalid start" << std::endl;

		if(goal_vertex == nullptr)
			std::cout << "Invalid goal" << std::endl;
	}
	else
	{
		clock_t		exec_time;
		exec_time = clock();
		path = sgrid_planner.Search(start_vertex_id, goal_vertex_id);
		exec_time = clock() - exec_time;
		std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
	}

	/*** Send data to Rviz ***/
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_lcm_msgs::Graph_t graph_msg = GetLcmGraphFromPlanner(sgrid_planner);
	lcm->publish("quad_planner/quad_planner_graph", &graph_msg);

	if(!path.empty())
	{
		srcl_lcm_msgs::Path_t path_msg;

		path_msg.waypoint_num = path.size();
		for(auto& wp : path)
		{
			srcl_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = wp->bundled_data_->location_.x;
			waypoint.positions[1] = wp->bundled_data_->location_.y;
			waypoint.positions[2] = 0.1;

			path_msg.waypoints.push_back(waypoint);
		}

		lcm->publish("quad_planner/quad_planner_graph_path", &path_msg);
	}

	/*** Visualize the map and graph ***/
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	Vis::VisSquareGrid(*sgrid_planner.map_.data_model, sgrid_planner.map_.padded_image, vis_img);

	/*** put the graph on top of the square grid ***/
	Vis::VisGraph(*sgrid_planner.graph_, vis_img, vis_img, true);
	/*** put the path on top of the graph ***/
	if(!path.empty())
		Vis::VisGraphPath(path, vis_img, vis_img);

	if(!show_padding)
	{
		Range rngx(0 + sgrid_planner.map_.info.padded_left, vis_img.cols - sgrid_planner.map_.info.padded_right);
		Range rngy(0 + sgrid_planner.map_.info.padded_top, vis_img.rows - sgrid_planner.map_.info.padded_bottom);

		// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
		vis_img = vis_img(rngy,rngx);
	}

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

//	imwrite( "new_map_path_cmp2.jpg", vis_result);

	return 0;
}
