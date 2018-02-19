/*
 * =====================================================================================
 *
 *       Filename:  test_navfield.cpp
 *
 *    Description:  :
 *
 *        Version:  1.0
 *        Created:  01/17/2017 03:25:40 PM
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
#include <ctime>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"
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

	if ( argc == 2 )
	{
		input_map = imread( argv[1], IMREAD_GRAYSCALE );

		if (!input_map.data)
		{
			printf("No image data \n");
			return -1;
		}
		else
		{
			sgrid_map = SGridBuilderV2::BuildSquareGridMap(input_map, 32,1);
			use_input_image = true;
		}
	}
	else{
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
	}

	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

//	Path_t<SquareCell*> path;
//	if(start_vertex == nullptr || finish_vertex == nullptr) {
//		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
//	}
//	else {
//		clock_t		exec_time;
//		exec_time = clock();
//		path = AStar::Search(graph,start_vertex,finish_vertex);
//		exec_time = clock() - exec_time;
//		std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
//	}

//	auto nbs = sgrid_map.data_model->GetNeighboursWithinRange(603, 1);
//	for(const auto& n : nbs)
//		std::cout << n->data_id_ << std::endl;

	///////////////////////////////////////////////////////////////

	std::shared_ptr<NavField<SquareCell*>> nav_field = std::make_shared<NavField<SquareCell*>>(graph);
	nav_field->UpdateNavField(107);
////	nav_field->UpdateNavField(95);
//////	nav_field.UpdateNavField(185); // 32
////	nav_field->UpdateNavField(185); // 32
////	//nav_field.UpdateNavField(60); // 64
//////	nav_field->UpdateNavField(50); // new lab map
//////	nav_field->UpdateNavField(406); // lab map
//////	nav_field->UpdateNavField(536); // case 3

	ShortcutEval sc_eval(sgrid_map.data_model, nav_field);
	sc_eval.EvaluateGridShortcutPotential(20);

	// abstract: 552, 95
	// case 3: 930, 536
	Vertex_t<SquareCell*> * start_vertex = graph->GetVertexFromID(107); // 390 for case 4// 552, 508
	Vertex_t<SquareCell*> * finish_vertex = graph->GetVertexFromID(704);//50 for case 4 //95

	auto nav_path = sc_eval.SearchInNavField(start_vertex, finish_vertex);
	//auto nav_path = sc_eval.SearchInNavFieldbyStep(start_vertex, finish_vertex);

	///////////////////////////////////////////////////////////////

	Mat vis_img;

	if(!use_input_image)
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

//	Vis::VisGraph(*graph, vis_img, vis_img, true);

	Vertex_t<SquareCell*>* check_vtx = graph->GetVertexFromID(704); // 390 for case 4// 552, 508
	Vis::VisSquareGridLocalNavField(*sgrid_map.data_model, *nav_field, check_vtx, vis_img, vis_img, 20);

//	Vis::VisSquareGridShortcutPotential(*nav_field, vis_img, vis_img);
//
//	if(!nav_path.empty())
//		Vis::VisGraphPath(nav_path, vis_img, vis_img);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE

	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite("potential_field.jpg", vis_img);

	return 0;
}
