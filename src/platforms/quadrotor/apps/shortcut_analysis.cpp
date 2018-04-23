/*
 * =====================================================================================
 *
 *       Filename:  shortcut_analysis.cpp
 *
 *    Description:  :
 *
 *        Version:  1.0
 *        Created:  04/27/2017 03:25:40 PM
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
#include "map/image_utils.h"

#include "path_repair/nav_field.h"
#include "path_repair/shortcut_eval.h"

#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/square_grid.h"

#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"

using namespace cv;
using namespace librav;


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
			sgrid_map = SGridBuilderV2::BuildSquareGridMap(input_map, 16,1);
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

//	auto nbs = sgrid_map.data_model->GetNeighboursWithinRange(603, 1);
//	for(const auto& n : nbs)
//		std::cout << n->data_id_ << std::endl;

	///////////////////////////////////////////////////////////////

	std::shared_ptr<NavField<SquareCell*>> nav_field = std::make_shared<NavField<SquareCell*>>(graph);
	nav_field->UpdateNavField(800);

	ShortcutEval sc_eval(sgrid_map.data_model, nav_field);
	sc_eval.EvaluateGridShortcutPotential(15);

	Vertex_t<SquareCell*> * start_vertex = graph->GetVertexFromID(2000);
	Vertex_t<SquareCell*> * finish_vertex = graph->GetVertexFromID(800);

	Path_t<SquareCell*> nav_path;
	if(start_vertex == nullptr || finish_vertex == nullptr)
		std::cout << "start or goal vertex is not valid" << std::endl;
	else
		nav_path = sc_eval.SearchInNavField(start_vertex, finish_vertex);

	///////////////////////////////////////////////////////////////

	Path_t<SquareCell*> geo_path;
	if(start_vertex == nullptr || finish_vertex == nullptr) {
		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
	}
	else {
		clock_t		exec_time;
		exec_time = clock();
		geo_path = AStar::Search(graph,start_vertex,finish_vertex);
		exec_time = clock() - exec_time;
		std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
	}

	///////////////////////////////////////////////////////////////

	Mat vis_img;

	if(!use_input_image)
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

//	Vis::VisGraph(*graph, vis_img, vis_img, true);
//
//	Vertex_t<SquareCell*>* check_vtx = graph->GetVertexFromID(1825); // 390 for case 4// 552, 508
//	Vis::VisSquareGridLocalNavField(*sgrid_map.data_model, *nav_field, check_vtx, vis_img, vis_img, 15);

	Vis::VisSquareGridShortcutPotential(*nav_field, vis_img, vis_img);

	if(!geo_path.empty())
		Vis::VisGraphPath(geo_path, vis_img, vis_img, Scalar( 66, 66, 244 ));

	if(!nav_path.empty())
		Vis::VisGraphPath(nav_path, vis_img, vis_img);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE

	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite("potential_field.jpg", vis_img);

	return 0;
}
