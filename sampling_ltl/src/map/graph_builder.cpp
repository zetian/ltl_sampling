/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include <map/graph_builder.h>
#include <iostream>
#include <cmath>

using namespace srcl;

std::shared_ptr<Graph_t<SquareCell*>> GraphBuilder::BuildFromSquareGrid(const std::shared_ptr<SquareGrid>& grid, bool allow_diag_move)
{
	std::shared_ptr<Graph<SquareCell*>> graph = std::make_shared<Graph<SquareCell*>>();

	for(auto itc = grid->cells_.begin(); itc != grid->cells_.end(); itc++)
	{
		uint64_t current_nodeid = (*itc).second->data_id_;

		if(grid->cells_[current_nodeid]->occu_ != OccupancyType::OCCUPIED) {
			std::vector<SquareCell*> neighbour_list = grid->GetNeighbours(current_nodeid,allow_diag_move);

			for(auto itn = neighbour_list.begin(); itn != neighbour_list.end(); itn++)
			{
				if(grid->cells_[(*itn)->data_id_]->occu_ != OccupancyType::OCCUPIED)
				{
					double error_x,error_y, cost = 0;
					error_x = std::abs(static_cast<long>((*itn)->location_.x) - static_cast<long>((*itc).second->location_.x));
					error_y = std::abs(static_cast<long>((*itn)->location_.y) - static_cast<long>((*itc).second->location_.y));
					cost = std::sqrt(error_x*error_x + error_y*error_y);

					graph->AddEdge((*itc).second, *itn, cost);
				}
			}
		}
	}

	return graph;
}

std::shared_ptr<Graph_t<SquareCell*>> GraphBuilder::BuildFromSquareGrid_IgnoreObstacle(const std::shared_ptr<SquareGrid> grid, bool allow_diag_move)
{
	auto graph = std::make_shared<Graph<SquareCell*>>();

	for(auto itc = grid->cells_.begin(); itc != grid->cells_.end(); itc++)
	{
		uint64_t current_nodeid = (*itc).second->data_id_;

		std::vector<SquareCell*> neighbour_list = grid->GetNeighbours(current_nodeid,allow_diag_move);

		for(auto itn = neighbour_list.begin(); itn != neighbour_list.end(); itn++)
		{
			double error_x,error_y, cost = 0;
			error_x = std::abs(static_cast<long>((*itn)->location_.x) - static_cast<long>((*itc).second->location_.x));
			error_y = std::abs(static_cast<long>((*itn)->location_.y) - static_cast<long>((*itc).second->location_.y));
			cost = std::sqrt(error_x*error_x + error_y*error_y);
			graph->AddEdge((*itc).second, *itn, cost);
		}
	}

	return graph;
}
