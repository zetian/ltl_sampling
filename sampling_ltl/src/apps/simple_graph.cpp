/*
 * simple_graph.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: rdu
 */


// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>

// user
#include "graph/graph.h"
#include "graph/bds_example.h"

using namespace srcl;

int main(int argc, char** argv )
{
	std::vector<BDSExample*> nodes;

	// create nodes
	for(int i = 0; i < 4; i++) {
		nodes.push_back(new BDSExample(i));
	}

	// create a graph
	Graph_t<BDSExample> graph;

	graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
	graph.AddEdge(*(nodes[0]), *(nodes[2]), 1.5);
	graph.AddEdge(*(nodes[1]), *(nodes[2]), 2.0);
	graph.AddEdge(*(nodes[2]), *(nodes[3]), 2.5);

	std::vector<Edge_t<BDSExample>> all_edges = graph.GetGraphEdges();

	for(auto e : all_edges)
		e.PrintEdge();

	auto vtx = graph.GetVertexFromID(0);
	std::vector<Vertex_t<BDSExample>*> neighbours = vtx->GetNeighbours();

	std::cout << "finding neighbours of vertex 0: " << std::endl;
	for(auto e : neighbours)
		std::cout << "neighbour id: " << e->vertex_id_ << std::endl;

	std::cout << "check neighbours of vertex 0: " << std::endl;
	std::cout << "vertex 1: " << vtx->CheckNeighbour(graph.GetVertexFromID(1)) << std::endl;
	std::cout << "vertex 3: " << vtx->CheckNeighbour(graph.GetVertexFromID(3)) << std::endl;

	for(auto& e : nodes)
		delete e;

	return 0;
}



