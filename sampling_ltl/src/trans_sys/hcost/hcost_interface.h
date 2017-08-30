/*
 * hcost_interface.h
 *
 *  Created on: 18 Nov 2016
 *      Author: zetian
 */

#ifndef SRC_TRANS_SYS_HCOST_INTERFACE_H_
#define SRC_TRANS_SYS_HCOST_INTERFACE_H_

#include <vector>
#include <map>
#include <memory>
#include <tuple>

#include "graph/graph.h"
#include "map/square_grid.h"
#include "trans_sys/hcost/hcost_tile_library.h"

namespace srcl
{

namespace HCost {

//double get_lifted_transition(int H, std::vector<Vertex<SquareCell>> tile_vertices_Vertex, std::vector<int> rgn_idx_next, std::map<int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL);
double get_lifted_transition(int H, Eigen::Matrix<int,Eigen::Dynamic,4> tile_vertices, std::vector<int> rgn_idx_next, std::map<unsigned int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL);


int zta02rgn_idx(std::vector<double> zta0);

std::map<unsigned int,std::shared_ptr<Hlevel>> hcost_preprocessing();
}

}



#endif /* SRC_TRANS_SYS_HCOST_INTERFACE_H_ */
