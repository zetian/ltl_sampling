/*
 * map_gen.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "planning/graph/graph.h"
#include "map/image_utils.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
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
			sgrid_map = SGridBuilderV2::BuildSquareGridMap(input_map, 16, 1);
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

	Mat vis_img;

	if(sgrid_map.padded_image.empty())
		Vis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		Vis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	Range rngx(0 + sgrid_map.info.padded_left, vis_img.cols - sgrid_map.info.padded_right);
	Range rngy(0 + sgrid_map.info.padded_top, vis_img.rows - sgrid_map.info.padded_bottom);

	// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
	Mat vis_img_no_padding = vis_img(rngy,rngx);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE

	if(show_padding)
		imshow("Processed Image", vis_img);
	else
		imshow("Processed Image", vis_img_no_padding);

	waitKey(0);

	imwrite( "map_gen_result.jpg", vis_img_no_padding);
	imwrite( "map_gen_result_padded.jpg", vis_img);

	return 0;
}


