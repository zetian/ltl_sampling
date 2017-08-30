/*
  * buchi_automata.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: rdu
 */

#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <bitset>
#include <cmath>

#include "trans_sys/buchi_automaton.h"

using namespace srcl;

std::shared_ptr<Graph_t<BuchiState>> BuchiAutomaton::CreateBuchiGraph(std::string ltl_str, std::vector<std::string> ltl_states)
{
	SpotHoaInterpreter ltl2ba_lib;

	// generate buchi data structure using ltl2ba library
	BAStruct ba = ltl2ba_lib.GetBuchi(ltl_str,ltl_states);

	std::vector<BuchiState> buchi_states;

	// create new buchi graph
	std::shared_ptr<Graph_t<BuchiState>>  buchi_graph = std::make_shared<Graph<BuchiState>>();
	for(int i = 0; i < ba.state_num; i++) {
		BuchiState buchi_state(i);
		buchi_state.alphabet_set = ba.alphabet_set;
		buchi_state.acc_state_idx = ba.acc_state_idx;
		buchi_state.init_state_idx_ = ba.init_state_idx;
//		buchi_state.region_to_id = ba.region_to_id;
		buchi_states.push_back(buchi_state);
	}

	for(int i = 0; i < ba.state_num; i++) {
		for(int j = 0; j < ba.state_num; j++)
		{
			std::vector<uint32_t> ref = ba.trans_con[i][j];
			if (!ba.trans_con[i][j].empty()) {
				buchi_graph->AddEdge(buchi_states[i], buchi_states[j], 1.0, ba.trans_con[i][j]);
			}
		}
	}

	return buchi_graph;
}
