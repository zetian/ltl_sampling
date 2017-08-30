/*
 * test_memory.cpp
 *
 *  Created on: July 14, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>

// user
#include "graph/graph.h"
#include "map/sgrid_builder.h"
#include "map/graph_builder.h"
#include "vis/graph_vis.h"

#include "trans_sys/buchi_automaton.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/product_automaton.h"

using namespace srcl;

int main(int argc, char** argv )
{
	/*** 1. Create a empty square grid ***/
	int row_num = 10;
	int col_num = 10;
	std::shared_ptr<SquareGrid>	grid = std::make_shared<SquareGrid>(row_num,col_num,50);

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
	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p3) && (<> p4) && ((<> p2) || (<> p5))";
	//	std::string ltl_formula = "([] p0) && ([] ! p1) && ( <> p4 ||<> p5 || <> p2 || <> p3)";
	std::vector<std::string> buchi_regions;
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p5");
	buchi_regions.push_back("p3");
	//	buchi_regions.push_back("p5");
	buchi_regions.push_back("p4");

	std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions);

	/*** 4. Construct a lifted graph ***/
	int HistoryH = 1;
	std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);

	/*** 5. Construct a product graph ***/
	std::shared_ptr<Graph_t<ProductState>> product_graph = ProductAutomaton::CreateProductAutomaton(lifted_graph,buchi_graph);

	/*** 6. Search in the product graph ***/
	// Set start node in original graph
	Vertex<SquareCell *> * start_node_origin = graph->GetVertexFromID(59);

	// Convert from original node to product node
	std::vector<Vertex<ProductState>*> start_product;
	for (auto it = start_node_origin->lifted_vertices_id_.begin(); it!=start_node_origin->lifted_vertices_id_.end();it++){
		start_product.push_back(product_graph->GetVertexFromID(lifted_graph->GetVertexFromID(*it)->product_vertex_id_));
	}
	std::vector<Vertex<ProductState>*> path = product_graph->AStarProductSearch(start_product,buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx);
	std::cout << buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx [0]<<" FINAL" << std::endl;

	// Map path in the product graph back to the square grid graph
	std::vector<Vertex<SquareCell*>*> path_origin;
	for (auto it = path.begin()+1; it != path.end(); it++){
		path_origin.push_back((*it)->bundled_data_.lifted_vertex_->bundled_data_.history.front());
	}
	path_origin.insert(path_origin.end(),path.back()->bundled_data_.lifted_vertex_->bundled_data_.history.begin()+1,path.back()->bundled_data_.lifted_vertex_->bundled_data_.history.end());

	return 0;
}

