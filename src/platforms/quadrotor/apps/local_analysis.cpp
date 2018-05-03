/*
 * local_analysis.cpp
 *
 *  Created on: Feb 9, 2017
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
#include "map/image_utils.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/square_grid.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"

using namespace cv;
using namespace librav;

#include "path_repair/nav_field.h"
#include "path_repair/shortcut_eval.h"

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
			sgrid_map = SGridBuilder::BuildSquareGridMap(input_map, 32);
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

	///////////////////////////////////////////////////////////////

	std::shared_ptr<NavField<SquareCell*>> nav_field = std::make_shared<NavField<SquareCell*>>(graph);
	nav_field->UpdateNavField(95);

	Vertex_t<SquareCell*> * checked_vertex = graph->GetVertexFromID(516);
	if(checked_vertex->bundled_data_->occu_ == OccupancyType::OCCUPIED) {
		std::cout << "Checked cell is occupied." << std::endl;
		return -1;
	}

	///////////////////////////////////////////////////////////////

	Mat vis_img;

	if(!use_input_image)
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	//GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);
	//Vis::VisSquareGridNavField(*sgrid_map.data_model, *nav_field, vis_img, vis_img, true);
	Vis::VisSquareGridLocalNavField(*sgrid_map.data_model, *nav_field, checked_vertex, vis_img, vis_img, 5);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE

	imshow("Processed Image", vis_img);

	waitKey(0);

	imwrite("potential_field.jpg", vis_img);

	return 0;
}
