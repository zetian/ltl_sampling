/*
 * graph_example.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "map/sgrid_builder.h"
#include "map/graph_builder.h"
#include "map/map_utils.h"
#include "vis/graph_vis.h"

#include "trans_sys/buchi_automaton.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/product_automaton.h"

using namespace cv;
using namespace srcl;

void test_product_2(Graph<ProductState>* product_graph){
	auto all_edges = product_graph->GetGraphEdges();
	for(const auto& edge: all_edges)
	{
		//		edge.PrintEdge();
		auto src_node = edge.src_;
		auto dst_node = edge.dst_;
		auto src_history = src_node->bundled_data_.lifted_vertex_->bundled_data_.history;
		auto dst_history = dst_node->bundled_data_.lifted_vertex_->bundled_data_.history;
		std::cout << "src history: ";
		for (auto it:src_history){
			std::cout << it->vertex_id_ << " ";
		}
		std::cout << std::endl;
		std::cout << "dst history: ";
		for (auto it:dst_history){
			std::cout << it->vertex_id_ << " ";
		}
		std::cout << std::endl;
		std::cout << "region id: ";
		//		auto last_cell_region = dst_node->bundled_data_.lifted_vertex_->bundled_data_.history.back()->bundled_data_.GetTaskRegionBitMap();
		//		std::cout <<last_cell_region<<std::endl;
		auto buchi_transition = (src_node->bundled_data_.buchi_vertex_)->GetBuchiTransition(dst_node->bundled_data_.buchi_vertex_);
		std::cout << "Admissible buchi transition is ";
		for (auto it:buchi_transition){
			std::cout << it << " ";
		}
		std::cout << std::endl;
		std::cout << std::endl;
	}


}

int main(int argc, char** argv )
{
	int row_num = 10;
	int col_num = 10;

	Mat input_image;
	bool use_input_image = false;

	Map_t<SquareGrid> sgrid_map;

	/*** check if user specifies an image ***/
	if ( argc == 2 )
	{
		input_image = imread( argv[1], IMREAD_GRAYSCALE );

		if (!input_image.data)
		{
			printf("No image data \n");
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
		std::shared_ptr<SquareGrid> grid = MapUtils::CreateSquareGrid(row_num,col_num,50);

		// set occupancy for cells
		for(auto it = 0; it < (row_num*col_num) ; it++){
			//			grid->cells_[it]->task_region_.SetRegionLabel(0);
			grid->cells_[it]->AssignRegionLabel(0);
		}
		//
		//		grid->cells_[2]->task_region_.SetRegionLabel(1);
		grid->cells_[2]->RemoveRegionLabel(0);
		grid->cells_[2]->AssignRegionLabel(1);
		grid->SetCellOccupancy(2, OccupancyType::OCCUPIED);
		grid->cells_[12]->RemoveRegionLabel(0);
		grid->cells_[12]->AssignRegionLabel(1);
		grid->SetCellOccupancy(12, OccupancyType::OCCUPIED);
		grid->cells_[22]->RemoveRegionLabel(0);
		grid->cells_[22]->AssignRegionLabel(1);
		grid->SetCellOccupancy(22, OccupancyType::OCCUPIED);
		grid->cells_[32]->RemoveRegionLabel(0);
		grid->cells_[32]->AssignRegionLabel(1);
		grid->SetCellOccupancy(32, OccupancyType::OCCUPIED);
		grid->cells_[48]->RemoveRegionLabel(0);
		grid->cells_[48]->AssignRegionLabel(1);
		grid->SetCellOccupancy(48, OccupancyType::OCCUPIED);
		grid->cells_[46]->RemoveRegionLabel(0);
		grid->cells_[46]->AssignRegionLabel(1);
		grid->SetCellOccupancy(46, OccupancyType::OCCUPIED);
		grid->cells_[47]->RemoveRegionLabel(0);
		grid->cells_[47]->AssignRegionLabel(1);
		grid->SetCellOccupancy(47, OccupancyType::OCCUPIED);

		grid->cells_[25]->AssignRegionLabel(2);
		grid->SetCellOccupancy(25, OccupancyType::INTERESTED);
		grid->cells_[71]->AssignRegionLabel(3);
		grid->SetCellOccupancy(71, OccupancyType::INTERESTED);
		grid->cells_[66]->AssignRegionLabel(4);
		grid->SetCellOccupancy(66, OccupancyType::INTERESTED);
		grid->cells_[99]->AssignRegionLabel(5);
		grid->SetCellOccupancy(99, OccupancyType::INTERESTED);

		sgrid_map.data_model = grid;
	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	auto graph = GraphBuilder::BuildFromSquareGrid_IgnoreObstacle(sgrid_map.data_model,false);
	//auto graph = GraphBuilder::BuildFromSquareGrid(grid,false);
	/*** Construct a graph from the buchi automata ***/
	BuchiAutomaton buchi_autaton;
	std::vector<std::string> buchi_regions;
	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p2) && (<> p3) && (<> p4) && (<> p5)";
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p3");
	buchi_regions.push_back("p4");
	buchi_regions.push_back("p5");
	auto buchi_graph = buchi_autaton.CreateBuchiGraph(ltl_formula,buchi_regions);

	/*** Start constructing the transition system ***/
	GraphLifter graph_lifter;
	int HistoryH = 1;

	clock_t		lift_time;
	lift_time = clock();
	auto lifted_graph = graph_lifter.CreateLiftedGraph(HistoryH, graph);
	lift_time = clock() - lift_time;
	std::cout << "Lift graph time in " << double(lift_time)/CLOCKS_PER_SEC << " s." << std::endl;

	clock_t		product_time;
	product_time = clock();
	auto product_graph = ProductAutomaton::CreateProductAutomaton(lifted_graph,buchi_graph);
	product_time = clock() - product_time;
	std::cout << "Product graph time in " << double(product_time)/CLOCKS_PER_SEC << " s." << std::endl;

	//	auto start_node = product_graph->GetVertexFromID(0);
	//	std::cout << start_node->bundled_data_.buchi_vertex_->vertex_id_ <<" in buchi" << std::endl;
	//	auto start_history = start_node->bundled_data_.lifted_vertex_->bundled_data_.history;
	//	for (auto it:start_history){
	//		std::cout << it->vertex_id_ << " ";
	//	}
	//	std::cout << std::endl;
	//	auto all_state_product = product_graph->GetGraphVertices();
	//	uint64_t final_state = 0;
	//	uint64_t buchi_f;
	//	buchi_f=0;
	//	for (auto it:all_state_product){
	//		buchi_f = it->bundled_data_.buchi_vertex_->vertex_id_;
	//		if (buchi_f == 3){
	//			final_state = it->vertex_id_;
	//			break;
	//		}
	//	}
	//	std::cout << "~~~" << final_state << std::endl;
	//	auto end_node = product_graph->GetVertexFromID(3583);

	//	std::vector<uint32_t> buchi_goal_id;
	//	buchi_goal_id.push_back(15);

	std::cout << "graph size: " << product_graph->GetGraphVertices().size() << std::endl;

	clock_t		exec_time;
	exec_time = clock();

	//Set start node in original graph
	auto start_node_origin = graph->GetVertexFromID(53);

	//Convert from original node to product node
	std::vector<Vertex<ProductState>*> start_product;
	for (auto it = start_node_origin->lifted_vertices_id_.begin(); it!=start_node_origin->lifted_vertices_id_.end();it++){
		start_product.push_back(product_graph->GetVertexFromID(lifted_graph->GetVertexFromID(*it)->product_vertex_id_));
	}

	std::vector<Vertex<ProductState>*> path = product_graph->AStarProductSearch(start_product,buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx);




	//std::vector<Vertex<ProductState>*> path = product_graph->AStarProductSearch(product_graph->GetVertexFromID(product_size),buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
	std::vector<Vertex<SquareCell*>*> path_origin;

	//draw the path in original graph
	for (auto it = path.begin()+1; it != path.end(); it++){
		path_origin.push_back((*it)->bundled_data_.lifted_vertex_->bundled_data_.history.front());
		//		path_origin.insert(path_origin.end(), (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.begin(), (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.end());
	}
	path_origin.insert(path_origin.end(),path.back()->bundled_data_.lifted_vertex_->bundled_data_.history.begin()+1,path.back()->bundled_data_.lifted_vertex_->bundled_data_.history.end());




	//	int HistoryH = 1;
	//
	//	GraphLifter graph_lifter;
	//	auto lifted_graph = graph_lifter.CreateLiftedGraph(HistoryH, graph);

	//	auto product_graph = ProductAutomatonBuilder::CreateProductAutomaton(lifted_graph,buchi_graph);


	//	int HistoryH = 3;
	//	//	std::vector<Vertex<SquareCell>*> rawSeq1;
	//	//	std::vector<std::vector<Vertex<SquareCell>*>> allHist;
	//	auto lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
	//================Test lifted graph========================
	//
	//	int node_id =36;
	//	std::cout <<"===Target node：" << std::endl;
	//	for(auto ite:(lifted_graph->GetVertexFromID(node_id)->bundled_data_.history)){
	//		std::cout << ite->vertex_id_ << " ";
	//	}
	//	std::cout << std::endl;
	//	auto neighbour = lifted_graph->GetVertexFromID(node_id)->GetNeighbours();
	//	std::cout << "neighbour num: " << neighbour.size() << std::endl;
	//	for (auto it:neighbour){
	//		std::cout << "neighbour id " << it->vertex_id_ << std::endl;
	//		for(auto ite:it->bundled_data_.history){
	//			std::cout << ite->vertex_id_ << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//=================================================================
	/*** Visualize the map and graph ***/
	GraphVis vis;
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	if(sgrid_map.padded_image.empty())
		vis.VisSquareGrid(*sgrid_map.data_model, vis_img);
	else
		vis.VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);

	/*** put the graph on top of the square grid ***/
	vis.VisSquareGridGraph(*graph, vis_img, vis_img, true);
	vis.VisSquareGridPath(path_origin, vis_img, vis_img);
	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);
	//
	waitKey(0);

	/*** uncomment this line if you want to save result into an image ***/
	// imwrite( "examples_result.jpg", vis_img);


	//	delete grid;
	//	delete graph;
	//	delete buchi_graph;
	//	delete lifted_graph;


	return 0;
}

