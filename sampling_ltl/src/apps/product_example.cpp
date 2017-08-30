// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "map/sgrid_builder.h"
#include "map/graph_builder.h"
#include "map/map_utils.h"
#include "vis/graph_vis.h"

#include "trans_sys/buchi_automaton.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/product_automaton.h"
#include "trans_sys/hcost/hcost_interface.h"

using namespace cv;
using namespace srcl;

int main(int argc, char** argv )
{
	/*** 0. Preprocessing CBTA data ***/
	std::map<unsigned int,std::shared_ptr<Hlevel>> h_levels = HCost::hcost_preprocessing();
	/*** 1. Create a empty square grid ***/
	int row_num = 10;
	int col_num = 10;

	std::shared_ptr<SquareGrid> grid = MapUtils::CreateSquareGrid(row_num,col_num,50);

	// assign properties of square cells
	for(auto it = 0; it < (row_num*col_num) ; it++){
		grid->cells_[it]->AssignRegionLabel(0);
	}

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

	grid->cells_[25]->AssignRegionLabel(2);
	grid->SetCellOccupancy(25, OccupancyType::INTERESTED);

	grid->cells_[71]->AssignRegionLabel(3);
	grid->SetCellOccupancy(71, OccupancyType::INTERESTED);

	grid->cells_[66]->AssignRegionLabel(4);
	grid->SetCellOccupancy(66, OccupancyType::INTERESTED);

	grid->cells_[99]->AssignRegionLabel(5);
	grid->SetCellOccupancy(99, OccupancyType::INTERESTED);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid_IgnoreObstacle(grid,false);

	/*** 3. Construct a graph from the buchi automata ***/
//	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p3) && (<> p4) && ((<> p2) || (<> p5))";
	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p3) && (<> p4) && (<> p2) && (<> p5)";
//	std::string ltl_formula = "[] p0 && [] !p1 && <> (p2 && <> p3)";
//	std::string ltl_formula = "([] p0) && ([] ! p1) && ( <> p4 ||<> p5 || <> p2 )";
	std::vector<std::string> buchi_regions;
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
//	buchi_regions.push_back("p5");
	buchi_regions.push_back("p3");
	buchi_regions.push_back("p5");
	buchi_regions.push_back("p4");

	std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions);

	/*** 4. Construct a lifted graph ***/
	int HistoryH = 3;
	std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);

	/*** 5. Construct a product graph ***/
	clock_t     total_time;
	clock_t		product_time;
	total_time = clock();
	product_time = clock();
//	std::shared_ptr<Graph_t<ProductState>> product_graph = ProductAutomaton::CreateProductAutomaton(lifted_graph,buchi_graph);
	std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();
	product_time = clock() - product_time;
//	std::cout << "Product graph time in " << double(product_time)/CLOCKS_PER_SEC << " s." << std::endl;

	/*** 6. Search in the product graph ***/
	// Set start node in original graph
	Vertex<SquareCell *> * start_node_origin = graph->GetVertexFromID(93);

	// Convert from original node to product node
	uint64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, lifted_graph, buchi_graph, product_graph_new);
//	uint64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, lifted_graph, buchi_graph, product_graph);

//	auto virtual_start = product_graph->GetVertexFromID(virtual_start_id);
	// Search in product graph
	std::vector<uint32_t>  buchi_acc = buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx;

	GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, h_levels, grid);


	std::vector<ProductState> path = AStar::ProductIncSearch(product_graph_new, virtual_start_id,buchi_acc, GetNeighbourBDSFunc_t<ProductState>(get_product_cbta_neighbour));
//	std::vector<Vertex<ProductState>*> path = AStar::ProductSearch(product_graph, virtual_start,buchi_acc);

	total_time = clock() - total_time;
	std::cout << "Total time in " << double(total_time)/CLOCKS_PER_SEC << " s." << std::endl;
	// For debug, check path detail
	//	std::cout << "location: "<<std::endl;
	//	for (auto it = path.begin()+1; it!=path.end();it++){
	//		std::cout <<"Product id: " << (*it)->bundled_data_.GetID() <<", history:";
	//			for (auto ite = (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.begin();ite != (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.end(); ite++){
	//				std::cout << " "<<(*ite)->bundled_data_->GetID();
	//			}
	//		std::cout << ", Buchi state: " <<  (*it)->bundled_data_.buchi_vertex_->bundled_data_.GetID();
	//		std::cout << std::endl;
	//	}

	// Map path in the product graph back to the square grid graph

	std::vector<Vertex<SquareCell*>*> path_origin;
	for (auto it = path.begin()+1; it != path.end(); it++){
		path_origin.push_back((*it).lifted_vertex_->bundled_data_.history.front());
	}
	path_origin.insert(path_origin.end(),path.back().lifted_vertex_->bundled_data_.history.begin()+1,path.back().lifted_vertex_->bundled_data_.history.end());

	/*** 7. Visualize the map and graph ***/
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*grid, vis_img);
	vis.VisSquareGridGraph(*graph, vis_img, vis_img, true);
	vis.VisSquareGridPath(path_origin, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	return 0;
}
