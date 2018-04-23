/*
 * =====================================================================================
 *
 *       Filename:  cmp_path_02.cpp
 *
 *    Description:  This experiment is used to verify two terms of cost function
 *
 *        Version:  1.0
 *        Created:  10/26/2017 08:43:40 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ruixiang Du (rdu), ruixiang.du@gmail.com
 *   Organization:  Worcester Polytechnic Institute
 *
 * =====================================================================================
 */

// standard libaray
#include <iostream>
#include <vector>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"
#include "geometry/square_grid.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "map/image_utils.h"

#include "path_repair/nav_field.h"
#include "path_repair/shortcut_eval.h"

#include "utility/stopwatch/stopwatch.h"

using namespace cv;
using namespace librav;

int main(int argc, char *argv[])
{
	Map_t<SquareGrid> sgrid_map;

	// create a empty grid
	std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(20, 30, 50);

	// set occupancy for cells
	// for (int r = 1; r <= 3; r++)
	// 	for (int c = 1; c <= 12; c++)
	// 		grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	// for (int r = 6; r <= 8; r++)
	// 	for (int c = 3; c <= 14; c++)
	// 		grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	// for (int r = 15; r <= 17; r++)
	// 	for (int c = 5; c <= 16; c++)
	// 		grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	// for (int r = 19; r <= 21; r++)
	// 	for (int c = 3; c <= 14; c++)
	// 		grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	// for (int r = 23; r <= 25; r++)
	// 	for (int c = 5; c <= 16; c++)
	// 		grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	///****///
	for (int r = 1; r <= 3; r++)
		for (int c = 6; c <= 17; c++)
			grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	for (int r = 6; r <= 8; r++)
		for (int c = 8; c <= 19; c++)
			grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	for (int r = 13; r <= 15; r++)
		for (int c = 6; c <= 19; c++)
			grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	for (int r = 19; r <= 21; r++)
		for (int c = 2; c <= 13; c++)
			grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	for (int r = 25; r <= 27; r++)
		for (int c = 1; c <= 12; c++)
			grid->SetCellOccupancy(c, r, OccupancyType::OCCUPIED);

	sgrid_map.data_model = grid;

	std::shared_ptr<Graph_t<SquareCell *>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model, true);

	///////////////////////////////////////////////////////////////

	std::shared_ptr<NavField<SquareCell *>> nav_field = std::make_shared<NavField<SquareCell *>>(graph);
	nav_field->UpdateNavField(589);

	ShortcutEval sc_eval(sgrid_map.data_model, nav_field);
	sc_eval.EvaluateGridShortcutPotential(5);

	Vertex_t<SquareCell *> *start_vertex = graph->GetVertexFromID(9);
	Vertex_t<SquareCell *> *finish_vertex = graph->GetVertexFromID(589);

	auto nav_path = sc_eval.SearchInNavField(start_vertex, finish_vertex);

	///////////////////////////////////////////////////////////////

	Path_t<SquareCell *> geo_path;
	if (start_vertex == nullptr || finish_vertex == nullptr)
	{
		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
	}
	else
	{
		stopwatch::StopWatch stop_watch;
		geo_path = AStar::Search(graph, start_vertex, finish_vertex);
		std::cout << "Searched in " << stop_watch.toc() << " s." << std::endl;
	}

	///////////////////////////////////////////////////////////////

	Mat vis_img;

	Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);

	//	Vis::VisGraph(*graph, vis_img, vis_img, true);
	//
	//	Vertex_t<SquareCell*>* check_vtx = graph->GetVertexFromID(1825); // 390 for case 4// 552, 508
	//	Vis::VisSquareGridLocalNavField(*sgrid_map.data_model, *nav_field, check_vtx, vis_img, vis_img, 15);

	Vis::VisSquareGridShortcutPotential(*nav_field, vis_img, vis_img);

	// nav path in blue, shortest path in orange
	if (!geo_path.empty())
		Vis::VisGraphPath(geo_path, vis_img, vis_img, Scalar(66, 66, 244));

	if (!nav_path.empty())
		Vis::VisGraphPath(nav_path, vis_img, vis_img);

	namedWindow("Processed Image", WINDOW_NORMAL); // WINDOW_AUTOSIZE

	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite("potential_field.jpg", vis_img);

	return 0;
}
