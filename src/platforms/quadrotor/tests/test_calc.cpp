/*
 * test_calc.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "vis/graph_vis.h"
#include "geometry/graph_builder.h"
#include "map/image_utils.h"
#include "geometry/sgrid_builder.h"

using namespace cv;
using namespace librav;

#include "path_repair/nav_field.h"
#include "path_repair/shortcut_eval.h"
#include "geometry/square_grid/square_grid.h"

int main(int argc, char* argv[])
{
	Mat input_map;
	bool use_input_image = false;
	bool show_padding = false;

	Map_t<SquareGrid> sgrid_map;

	// create a empty grid
	std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(25,25,95);

	// set occupancy for cells
	for(int i = 425; i <= 440; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 450; i <= 465; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 184; i <= 199; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 209; i <= 224; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 234; i <= 249; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	sgrid_map.data_model = grid;

	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	Vertex_t<SquareCell*> * start_vertex = graph->GetVertexFromID(508);//552
	Vertex_t<SquareCell*> * finish_vertex = graph->GetVertexFromID(95);

	std::shared_ptr<NavField<SquareCell*>> nav_field = std::make_shared<NavField<SquareCell*>>(graph);
	//nav_field.UpdateNavField(185); // 32
	//nav_field.UpdateNavField(60); // 64
	nav_field->UpdateNavField(95);

	ShortcutEval sc_eval(sgrid_map.data_model, nav_field);
	//auto nav_path = nav_field.SearchInNavField(start_vertex, finish_vertex);
	auto nav_path = sc_eval.SearchInNavField(start_vertex, finish_vertex);

	std::cout << "dist: " << sc_eval.CalcDirectDistance(Position2Di(0,3), Position2Di(3,0), 5, false) << std::endl;
	std::cout << "dist: " << sc_eval.CalcDirectDistance(Position2Di(0,3), Position2Di(2,0), 5, false) << std::endl;

	std::cout << "dist: " << sc_eval.CalcDirectDistance(Position2Di(0,3), Position2Di(3,0), 5, true) << std::endl;
	std::cout << "dist: " << sc_eval.CalcDirectDistance(Position2Di(0,3), Position2Di(2,0), 5, true) << std::endl;

	return 0;
}
