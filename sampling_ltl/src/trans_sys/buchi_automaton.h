/*
 * buchi_automata.h
 *
 *  Created on: Feb 16, 2016
 *      Author: rdu
 */

#ifndef SRC_H2C_BUCHI_AUTOMATON_H_
#define SRC_H2C_BUCHI_AUTOMATON_H_

#include <cstring>
#include <cstdint>
#include <vector>
#include <memory>

#include "graph/graph.h"
#include "graph/bds_base.h"
#include "map/task_region.h"
#include "trans_sys/spot_hoa_interpreter.h"

namespace srcl{

/// BuchiState is used to construct graph of buchi automata.
struct BuchiState: public BDSBase<BuchiState>{

	BuchiState(uint64_t id):BDSBase<BuchiState>(id),init_state_idx_(0){};
	~BuchiState(){};

	std::vector<uint32_t> alphabet_set;
	std::vector<uint32_t> acc_state_idx;
	uint64_t init_state_idx_;
//	std::map<spot::formula, int> region_to_id;

	double GetHeuristic(const BuchiState& other_struct) const {
		return 0.0;
	}
};

/// BuchiAutomata class contains functions to create a Buchi graph from LTL statement.
namespace BuchiAutomaton{

std::shared_ptr<Graph_t<BuchiState>> CreateBuchiGraph(std::string ltl_str, std::vector<std::string> ltl_states);

};

}

#endif /* SRC_H2C_BUCHI_AUTOMATON_H_ */
