/*
 * graph_builder.h
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_BUILDER_H_
#define SRC_GRAPH_GRAPH_BUILDER_H_

#include <memory>

#include "graph/graph.h"
#include "map/square_grid.h"

namespace srcl {

namespace GraphBuilder
{
	std::shared_ptr<Graph_t<SquareCell*>> BuildFromSquareGrid(const std::shared_ptr<SquareGrid>& grid, bool allow_diag_move);
	std::shared_ptr<Graph<SquareCell*>> BuildFromSquareGrid_IgnoreObstacle(const std::shared_ptr<SquareGrid> grid, bool allow_diag_move);
};
}

#endif /* SRC_GRAPH_GRAPH_BUILDER_H_ */
