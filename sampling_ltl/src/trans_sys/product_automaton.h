/*
 * product_automaton.h
 *
 *  Created on: 5 May 2016
 *      Author: Zetian
 */

#ifndef SRC_H2C_PRODUCT_AUTOMATON_H_
#define SRC_H2C_PRODUCT_AUTOMATON_H_

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>

//#include "map/square_grid.h"
#include "graph/graph.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/buchi_automaton.h"
#include "trans_sys/hcost/hcost_tile_library.h"

namespace srcl
{

struct ProductState: public BDSBase<ProductState>
{
	ProductState(uint64_t id):
		BDSBase<ProductState>(id),
		lifted_vertex_(nullptr),
		buchi_vertex_(nullptr){};
	~ProductState(){};

	Vertex<LiftedSquareCell>* lifted_vertex_;
	Vertex<BuchiState>* buchi_vertex_;
	double GetHeuristic(const ProductState& other_struct) const
	{
		return 0.0;
	}
};

namespace ProductAutomaton
{

std::shared_ptr<Graph_t<ProductState>> CreateProductAutomaton(std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph);
uint64_t SetVirtualStart(Vertex<SquareCell *> * start_node_origin, std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph, std::shared_ptr<Graph_t<ProductState>> product_graph);
uint64_t SetVirtualStartIncre(Vertex<SquareCell *> * start_node_origin, std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph, std::shared_ptr<Graph_t<ProductState>> product_graph);

};

class GetProductCBTANeighbour {
public:
	GetProductCBTANeighbour(std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph, std::map<unsigned int,std::shared_ptr<Hlevel>> h_levels, std::shared_ptr<SquareGrid> grid){
		lifted_graph_ = lifted_graph;
		buchi_graph_ = buchi_graph;
		h_levels_ = h_levels;
		grid_ = grid;
	}

private:
	std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph_;
	std::shared_ptr<Graph_t<BuchiState>> buchi_graph_;
	std::map<unsigned int,std::shared_ptr<Hlevel>> h_levels_;
	std::shared_ptr<SquareGrid> grid_;

public:
	std::vector<std::tuple<ProductState, double>> operator()(ProductState cell);
};


}

#endif /* SRC_H2C_PRODUCT_AUTOMATON_H_ */
