/*
 * example.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "vis/graph_vis.h"
#include "map/graph_builder.h"
#include "map/sgrid_builder.h"
#include "map/map_type.h"
#include "map/map_utils.h"

using namespace cv;
using namespace srcl;

int main(int argc, char** argv )
{
	Mat input_image;
	bool use_input_image = false;

	Map_t<SquareGrid> sgrid_map;

	/*** check if user specifies an image ***/
	if ( argc == 2 )
	{
		std::string path = argv[1];
		bool read_result = MapUtils::ReadImageFromFile(path, input_image);

		if (!read_result)
		{
			printf("Failed to get image data from file \n");
			return -1;
		}
		/*** create a square grid map from input image ***/
		else
		{
			/*** BuildSquareGridMap() returns both the square grid data structure    ***/
			/***  and the post-processed image, this image is usually not the same   ***/
			/***  with the original input image (after binarizing, padding). The     ***/
			/***  square grid or the graph can be visualized over this image without ***/
			/***  mis-placement. ***/
			sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);

			/*** BuildSquareGrid() only returns the square grid data structure ***/
			//grid = SGridBuilder::BuildSquareGrid(input_image, 32);

			use_input_image = true;
		}
	}
	/*** otherwise, create a square grid map manually ***/
	else{
		// create a empty grid
		std::shared_ptr<SquareGrid> grid = MapUtils::CreateSquareGrid(12,12,95);

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

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,false);

	/*** Search path in the graph ***/
	Vertex_t<SquareCell*> * start_vertex;
	Vertex_t<SquareCell*> * finish_vertex;
	if(use_input_image)
	{
		start_vertex = graph->GetVertexFromID(160);
		finish_vertex = graph->GetVertexFromID(830);
	}
	else
	{
		start_vertex = graph->GetVertexFromID(0);
		finish_vertex = graph->GetVertexFromID(143);
	}

	if(start_vertex == nullptr || finish_vertex == nullptr) {
		std::cerr << "Invalid starting and finishing vertices, please choose two vertices in free space!" << std::endl;
		std::cerr << "Use image \"example.png\" inside \\planning\\data folder for this demo." << std::endl;
		return 0;
	}

	clock_t		exec_time;
	exec_time = clock();
	auto path = AStar::Search(graph, start_vertex,finish_vertex);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	/*** Visualize the map and graph ***/
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	if(sgrid_map.padded_image.empty())
		GraphVis::VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		GraphVis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	/*** put the graph on top of the square grid ***/
	GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);
		/*** put the path on top of the graph ***/
	GraphVis::VisSquareGridPath(path, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	/*** uncomment this line if you want to save result into an image ***/
	// imwrite( "examples_result.jpg", vis_img);

	return 0;
}
