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
			sgrid_map = SGridBuilder::BuildSquareGridMap(input_map, 64);
			use_input_image = true;
		}
	}
	else{
		// create a empty grid
		std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(12,12,95);

		// set occupancy for cells
		for(int i = 52; i <= 57; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 88; i <= 93; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 74; i <= 75; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		for(int i = 0; i < 8; i++)
			grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

		for(int i = 24; i <= 28; i++)
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(58, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(87, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(22, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(34, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(46, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(118, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(119, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(7, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(19, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(31, OccupancyType::OCCUPIED);

		grid->SetCellOccupancy(66, OccupancyType::OCCUPIED);
		grid->SetCellOccupancy(81, OccupancyType::OCCUPIED);

		sgrid_map.data_model = grid;
	}

	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	Vertex_t<SquareCell*> * start_vertex = graph->GetVertexFromID(225);
	Vertex_t<SquareCell*> * finish_vertex = graph->GetVertexFromID(60);

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

	///////////////////////////////////////////////////////////////

	NavField<SquareCell*> nav_field(graph);
	//nav_field.UpdateNavField(185); // 32
	//nav_field.UpdateNavField(60); // 64
	//auto nav_path = nav_field.SearchInNavField(start_vertex, finish_vertex);

	///////////////////////////////////////////////////////////////

	Mat vis_img;

	if(!use_input_image)
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	Vis::VisGraph(*graph, vis_img, vis_img, false);

//	if(!nav_path.empty())
//		GraphVis::VisSquareGridPath(nav_path, vis_img, vis_img);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE

	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite("potential.jpg", vis_img);

	return 0;
}
