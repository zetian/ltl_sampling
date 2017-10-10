/*
 * cbba_Agent_sampling.h
 *
 *  Created on: Oct 4, 2017
 *      Author: jfang
 */

#ifndef SRC_TRANS_SYS_CBBA_AGENT_SAMPLING_H_
#define SRC_TRANS_SYS_CBBA_AGENT_SAMPLING_H_

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "trans_sys/ltl_formula_sampling.h"

namespace acel{

// The number of agents
// const int N = 4;
// The number of tasks
// const int M = 6;
// The number of helpers required
// const int N_helper = 2;

const float eps = 0;
// const int max_bundle_length = M;

/*** General CBBA ***/
typedef struct
{
	std::vector<std::vector<int>> x_history;
	std::vector<std::vector<float>> y_history;
	std::vector<std::vector<int>> z_history;
	std::vector<std::vector<int>> iter_neighbors_his;
}Memo;

class cbba_Agent
{
public:
	// Constructor
	cbba_Agent(int id, std::vector<int> com,
			std::vector<float> y,std::vector<int> z,std::vector<std::vector<float>> y_his,std::vector<std::vector<int>> z_his);

	// Destructor
	~cbba_Agent(){};

public:
	// The index of agent (agent 0,1,...)
	double radius_L_;
	double radius_R_;
	std::vector<double> init_state_;
	int Index;
	int start_nodes_id_;
	// The award of agent
	std::vector<float> cbba_award;
	// The set of available task
	std::vector<int> h_avai;
	// Bundle
	std::vector<int> cbba_bundle;
	// Path
	std::vector<int> cbba_path;
	// Iteration that the agent talk to neighbors
	std::vector<int> iteration_neighbors;
	// The highest bid for certain task that agent knows
	std::vector<float> cbba_y;
	// The winner for certain task that agent knows
	std::vector<int> cbba_z;
	// The assignment of task
	std::vector<int> cbba_x;

	std::vector<std::vector<double>> traj_;

	Memo history;

	// Communication Topology
	std::vector<int> communication;
	// The neighbor of the agent
	std::vector<int> neighbors;

	int Iter;


public:

	void assignment_update();
	std::vector<int> award_update_for_sampling(LTLFormula Global_LTL);
};

// namespace CBBA
// {

//     float PathLengthCalculation(std::string ltl_new, int agent_idx_);
//     void neighbor_finder(std::vector<cbba_Agent>& agent);
// 	void communicate(std::vector<cbba_Agent>& agent);
// 	void available_tasks_finder(cbba_Agent& agent_sig);
// 	int desired_task_finder(cbba_Agent& agent_sig);
// 	void bundle_remove(std::vector<cbba_Agent>& agent);
// 	void path_remove(std::vector<cbba_Agent>& agent);
// 	void bundle_add_for_sampling(std::vector<cbba_Agent>& agent, LTLFormula Global_LTL);
// 	bool success_checker(std::vector<cbba_Agent> agent);
// };

}



#endif /* SRC_TRANS_SYS_CBBA_AGENT_SAMPLING_H_ */
