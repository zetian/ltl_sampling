#ifndef SRC_H2C_GRAPH_LIFTER_H_
#define SRC_H2C_GRAPH_LIFTER_H_

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>

#include "map/square_grid.h"
#include "graph/graph.h"

namespace srcl{

struct LiftedSquareCell: public BDSBase<LiftedSquareCell>
{
	LiftedSquareCell(uint64_t id):BDSBase<LiftedSquareCell>(id){};
	~LiftedSquareCell(){};

	std::vector<Vertex<SquareCell*>*> history;

	double GetHeuristic(const LiftedSquareCell& other_struct) const {
		return 0.0;
	}
};

namespace GraphLifter{

std::shared_ptr<srcl::Graph_t<LiftedSquareCell>> CreateLiftedGraph(int historyH, std::shared_ptr<Graph_t<SquareCell*>> original_graph);
std::vector<std::vector<Vertex_t<SquareCell*>*>> GetHistories(Vertex_t<SquareCell*>* actNode, int historyH);

};

}

#endif /* SRC_H2C_GRAPH_LIFTER_H_ */
