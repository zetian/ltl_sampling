#include "sampling/cbba_sampling.h"


using namespace acel;


CBBA_sampling::CBBA_sampling(){
    num_agent_ = 0;
}

void CBBA_sampling::set_global_ltl(LTLFormula formula){
    Global_LTL_ = formula;
	num_tasks_ = formula.Num_Tasks;
	max_bundle_length = num_tasks_;
}

void CBBA_sampling::set_buchi_regions(std::vector<std::string> buchi_regions){
    buchi_regions_ = buchi_regions;
}

void CBBA_sampling::set_indep_set(std::vector<int> indep_set){
    indep_set_ = indep_set;
}

void CBBA_sampling::set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, int interest_id) {
    Region interest_region;
    interest_region.set_position(position_x, position_y);
    interest_region.set_region_interest(interest_id);
    all_interest_regions_.push_back(interest_region);
    all_interest_regions_map_[interest_id] = interest_region;
}


void CBBA_sampling::set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y){
    Region obstacle;
    obstacle.set_position(position_x, position_y);
    all_obstacles_.push_back(obstacle);
}

void CBBA_sampling::init_workspace(double work_space_size_x, double work_space_size_y){
    work_space_size_x_ = work_space_size_x;
    work_space_size_y_ = work_space_size_y;
}

void CBBA_sampling::init_parameter(int iteration_cbba, double EPSILON, double RADIUS){
	iteration_cbba_ = iteration_cbba;
	EPSILON_ = EPSILON;
    RADIUS_ = RADIUS;
    // radius_L_ = radius_L;
    // radius_R_ = radius_R;
}

void CBBA_sampling::add_agent(cbba_Agent agent){
    all_agent_.push_back(agent);
	num_agent_++;
	// Vis
	lcm::LCM lcm;
	sampling::sample_data node_data;
	node_data.state[0] = agent.init_state_[0];
	node_data.state[1] = agent.init_state_[1];
	lcm.publish("SAMPLE", &node_data);
}


double CBBA_sampling::path_length_calculation(std::string ltl_new, cbba_Agent& agent){
    std::vector<double> init_state = agent.init_state_;;
    // Set up the class
    LTL_SamplingDubins ltl_sampling_dubins;
    // std::vector<std::string> buchi_regions;
    // std::vector<int> indep_set;
	// std::vector<std::string> buchi_regions_indep = LTLDecomposition::ObtainBuchiRegion({ltl_new}).front();
	
    // std::vector<std::string> indep_set_str = buchi_regions_indep;
    // for (int i = 0; i < indep_set_str.size(); i++) {
    //     indep_set_str[i].erase(indep_set_str[i].begin());
	// 	indep_set.push_back(std::stoi(indep_set_str[i]));
	// }

	// std::cout << "======================debug===========================" << std::endl;
    ltl_sampling_dubins.read_formula(ltl_new, buchi_regions_, indep_set_);
    ltl_sampling_dubins.init_workspace(work_space_size_x_, work_space_size_y_);
    ltl_sampling_dubins.init_parameter(EPSILON_, RADIUS_, agent.radius_L_, agent.radius_R_);
    for (int i = 0; i < all_interest_regions_.size(); i++) {
        std::pair <double, double> position_x = all_interest_regions_[i].get_x_position();
        std::pair <double, double> position_y = all_interest_regions_[i].get_y_position();
        ltl_sampling_dubins.set_interest_region(position_x, position_y, all_interest_regions_[i].get_region_interest());
    }

    for (int i = 0; i < all_obstacles_.size(); i++) {
        std::pair <double, double> position_x = all_obstacles_[i].get_x_position();
        std::pair <double, double> position_y = all_obstacles_[i].get_y_position();
        ltl_sampling_dubins.set_obstacle(position_x, position_y);
    }

    ltl_sampling_dubins.set_init_state(agent.init_state_);
    
    // Set the number of iterations
    // int iterations = 300;
    
	// Start sampling searching
	std::cout << "Start searching: " << ltl_new << " for agent " << agent.Index << std::endl;
    ltl_sampling_dubins.start_sampling(iteration_cbba_);


	double path_length = ltl_sampling_dubins.get_path_length();

	std::cout << "sub task path length is: " << path_length << std::endl;

	return path_length;
};

// Find the neighbors of agent and update neighbors
void CBBA_sampling::neighbor_finder()
{
	for (auto it = all_agent_.begin(); it != all_agent_.end(); it++){
		(*it).neighbors.clear();
		for (int i = 0; i < all_agent_.size(); i++)
			if ((*it).communication[i] == 1 && (*it).Index != i)
				(*it).neighbors.push_back(all_agent_[i].Index);
			else
				continue;
	}

//	std::cout << "Check out the neighbors" << std::endl;
//	for (auto it1 = agent.begin(); it1 != agent.end(); it1++){
//		for (auto it2 = (*it1).neighbors.begin(); it2 != (*it1).neighbors.end(); it2++)
//			std::cout << (*it2) << " ";
//	    std::cout << std::endl;
//	}
};

// Let agent communicate with its neighbors
void CBBA_sampling::communicate()
{
	std::cout << "communicating ... " << std::endl;
	CBBA_sampling::neighbor_finder();
	// sender: itself k
	// receiver: i
	// task : j

	for (auto it_ag = all_agent_.begin(); it_ag != all_agent_.end(); it_ag++){
		for (int i = 0; i < (*it_ag).neighbors.size();i++){
			for (int j = 0; j < num_tasks_; j++){
				// Entries 1 to 4
				// if current agent k thinks that the winner of task j is itself k
				if ((*it_ag).history.z_history.back()[j] == (*it_ag).Index){

					/***************************************** Entry 1 ***************************************/
					// Entry 1: Update or leave
					// If the receiver (neighbor) i thinks the winner of task j is also itself i
					if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).neighbors[i]){
						// Update
						if ((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j] > eps){
							//std::cout << "case 1" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
					    // Equal score: require to break the tie
						else if(std::abs((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
							// select the winner of task j as the agent with smaller index
							if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
								//std::cout << "case 2" << std::endl;
								all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
								all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
							}
						}
					}

					/***************************************** Entry 2 ***************************************/
					// Entry 2: Update
					// If the receiver i thinks the winner of task j is also agent k (sender)
					// Update
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
						//std::cout << "case 3" << std::endl;
						all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
						all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
					}

					/***************************************** Entry 3 ***************************************/

					// Entry 3: Update or Leave
					// If the receiver i thinks the winner of task j is not k or itself i but other agent
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
						// Compare the iteration of task j, find which one is the least information of task j
						// Update
						if ((*it_ag).history.iter_neighbors_his.back()[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]]){
							//std::cout << "case 4" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
						// Update
						else if((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j] > eps){
							//std::cout << "case 5" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
						// Equal scores: break the tie by selecting the winner as the agent with smaller index
						else if ((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j] <= eps){
							if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
								//std::cout << "case 6" << std::endl;
								all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
								all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
							}
						}
					}

					/***************************************** Entry 4 ***************************************/
					// Entry 4: Update
					// If the agent i (receiver) has no idea about the winner
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == -1){
						//std::cout << "case 7" << std::endl;
						all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
						all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
					}

					else
						std::cout << "Unknown winner value 1" << std::endl;
				}



		        /*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/

				// Entries 5 to 8
				// If current agent i (sender) thinks the winner of task j is agent k (receiver)
				else if ((*it_ag).history.z_history.back()[j] == all_agent_[(*it_ag).neighbors[i]].Index){

					/***************************************** Entry 5 ***************************************/
					// Entry 5
				    // if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
				    // Leave
				    if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == all_agent_[(*it_ag).neighbors[i]].Index)
				    	std::cout << "Do nothing Entry 5" << std::endl;

				    /***************************************** Entry 6 ***************************************/
				    // Entry 6
					// If agent i (receiver) thinks the winner of task j is agent k (sender)
					// Reset (Because the agent k will definitely be the first one to know its own information )
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
						//std::cout << "case 7" << std::endl;
						all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = -1;
						all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = -1;
					}

				    /***************************************** Entry 7 ***************************************/
				    // Entry 7
					// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
						// Compare the iteration of agent k and i, find which one has the least information
						// Reset
						if ((*it_ag).history.iter_neighbors_his.back()[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]]){
							// agent k should have the updated information
							//std::cout << "case 8" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = -1;
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = -1;
						}
					}

				    /***************************************** Entry 8 ***************************************/
				    // Entry 8
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == -1)
						std::cout <<"Do nothing Entry 8" << std::endl;

					else
						std::cout << "Unknown winner value 2" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 9 to 13
				// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
				else if ((*it_ag).history.z_history.back()[j] >= 0){
					/***************************************** Entry 9 ***************************************/
					// Entry 9
					// if agent i (receiver) thinks the winner of task j should be itself i
					if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == all_agent_[(*it_ag).neighbors[i]].Index){
						// compare the iteration that agent k and i talk to the winner that agent k thinks
						// If agent k (sender) has the least information
						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
							// Update
							if ((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j] > eps){
								//std::cout << "case 8" << std::endl;
								all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
								all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
							}
							// If we have a tie: break the tie by selecting the agent with smaller index as the winner
							else if (std::abs((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
								if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
									// Update
									//std::cout << "case 9" << std::endl;
									all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
									all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
								}
							}
						}
					}

					/***************************************** Entry 10 ***************************************/
					// Entry 10
					// if agent i (receiver) thinks the winner of task j is agent k (sender)
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
						// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
							//std::cout << "case 10" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
						else{
							// Reset
							//std::cout << "case 11" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = -1;
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = -1;
						}
					}

					/***************************************** Entry 11 ***************************************/
					// Entry 11
					// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).history.z_history.back()[j]){
						// Update
						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
							//std::cout << "case 12" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
					}

					/***************************************** Entry 12 ***************************************/
					// Entry 12
					// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
						if ((*it_ag).history.iter_neighbors_his.back()[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]]){
							// Update
							if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] >= all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
								//std::cout << "case 13" << std::endl;
								all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
								all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
							}
							// Reset
							else if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] < all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
								//std::cout << "case 14" << std::endl;
								all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = -1;
								all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = -1;
							}
							else
								std::cout << "Should not be here Entry 12" << std::endl;
						}
						else{
							if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
								// Update
								if ((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j] > eps){
									//std::cout << "case 15" << std::endl;
									all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
									all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
								}
								// If we have tie, break the tie by selecting the agent as the winner with smaller index
								else if (std::abs((*it_ag).history.y_history.back()[j] - all_agent_[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
									if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
										//std::cout << "case 16" << std::endl;
										all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
										all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
									}
								}
							}
						}
					}
					/***************************************** Entry 13 ***************************************/
					// Entry 13
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == -1){
						// Update
						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
							//std::cout << "case 17" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
					}
					else
						std::cout << "Unknown winner value Entry 13" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 14 to 17
				else if ((*it_ag).history.z_history.back()[j] == -1){

					/***************************************** Entry 14 ***************************************/
					// Entry 14
					// Leave
					if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == all_agent_[(*it_ag).neighbors[i]].Index)
						std::cout << "Do nothing Entry 14" << std::endl;

					/***************************************** Entry 15 ***************************************/
					// Entry 15
					// Update
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
						//std::cout << "case 18" << std::endl;
						all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
						all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
					}

					/***************************************** Entry 16 ***************************************/
					// Entry 16
					// Update
					else if (all_agent_[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
						// Update
						if ((*it_ag).history.iter_neighbors_his.back()[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]] > all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[all_agent_[(*it_ag).neighbors[i]].cbba_z[j]]){
							//std::cout << "case 19" << std::endl;
							all_agent_[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
							all_agent_[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
						}
					}

					/***************************************** Entry 17 ***************************************/
					// Entry 17
					// Leave
					else if(all_agent_[(*it_ag).neighbors[i]].cbba_z[j] == -1)
						std::cout<< "Do noting Entry 17" << std::endl;

					else
						std::cout << "Unknown winner value Entry 17" <<std::endl;
				}

				else
					std::cout << "Unknown winner value end of communicate" <<std::endl;
			}

			for (int n = 0; n < num_agent_; n++){
				if (n != (*it_ag).neighbors[i] && all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[n] < (*it_ag).history.iter_neighbors_his.back()[n]){
					all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[n] = (*it_ag).history.iter_neighbors_his.back()[n];
				}
			}

			all_agent_[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).Index] = (*it_ag).Iter;
		}

	}
};

// Find the available tasks for agent
void CBBA_sampling::available_tasks_finder(cbba_Agent& agent_sig){
	//Initialize the available tasks
	agent_sig.h_avai.clear();
	bool condition_1;
	bool condition_2;

	for (int j = 0; j < num_tasks_; j++){

		// Initialize the condition for each task
		condition_1 = 0;
		condition_2 = 0;

		if(agent_sig.cbba_award[j] - agent_sig.cbba_y[j] > eps)
			condition_1 = 1;
		else if (std::abs(agent_sig.cbba_award[j] - agent_sig.cbba_y[j]) <= eps)
			if (agent_sig.Index < agent_sig.cbba_z[j])
				condition_2 = 1;

		if (condition_1 == 1 || condition_2 == 1)
			agent_sig.h_avai.push_back(j);
	}

//		std::cout << "The available task is" <<std::endl;
//		for (int j = 0; j < h_avai.size();j++)
//			std::cout << h_avai[j] << ' ';
//		std::cout << std::endl;
};


// Find the desired task for agent
int CBBA_sampling::desired_task_finder(cbba_Agent& agent_sig){
	int max = 0;
	int desired_Index = -1;

	// Update the available tasks for agent
	//std::cout << "P1" <<std::endl;
	CBBA_sampling::available_tasks_finder(agent_sig);

	if (!agent_sig.h_avai.empty()){
		// If for certain agent, there are more than one desired tasks, select the one with smaller task index
		for (int j = 0; j < agent_sig.h_avai.size();j++)
			if (agent_sig.cbba_award[agent_sig.h_avai[j]] > max){
		    	max = agent_sig.cbba_award[agent_sig.h_avai[j]];
		    	desired_Index = agent_sig.h_avai[j];
			}
	}

	return desired_Index;
};


// Remove the task from bundle and all the tasks added after it
void CBBA_sampling::bundle_remove(){
	std::cout << "bundle removing ... " << std::endl;

	for (auto it_ag = all_agent_.begin(); it_ag != all_agent_.end(); it_ag++){
		bool outbidForTask = 0;
		if (!(*it_ag).cbba_bundle.empty()){
			// Check whether agent is outbid by other agent for the tasks which are in the bundle
			for (int k = 0; k < (*it_ag).cbba_bundle.size(); k++){
				if ((*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] != (*it_ag).Index)
					outbidForTask = 1;

				// Remove the tasks added after the outbid task
				if (outbidForTask == 1){
					if ((*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] == (*it_ag).Index){
						(*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] = -1;
						(*it_ag).cbba_y[(*it_ag).cbba_bundle[k]] = -1;
					}

					// Once the path is taken into consideration
				    // we need to find the position of task in the path

				    // remove the task from the bundle
					(*it_ag).cbba_bundle[k] = -1;
				}
			}
		}

		// Remove the task which has -1
		(*it_ag).cbba_bundle.erase(std::remove((*it_ag).cbba_bundle.begin(),(*it_ag).cbba_bundle.end(),-1),(*it_ag).cbba_bundle.end());


		std::cout << "The cbba_bunlde after bundle_remove is " << std::endl;
		for (int i = 0; i < (*it_ag).cbba_bundle.size();i++)
			std::cout << (*it_ag).cbba_bundle[i] <<std::endl;

		// Remove the tasks which are not showed in the bundle
		//agent = path_remove(std::vector<cbba_Agent> agent);
	}

	CBBA_sampling::path_remove();
};

void CBBA_sampling::path_remove(){

	for(auto it_ag = all_agent_.begin(); it_ag != all_agent_.end(); it_ag++){
		std::vector<bool> existence;
		existence.clear();
		for (int m = 0; m < (*it_ag).cbba_path.size(); m++)
			existence.push_back(0);

		// For test
	//	std::cout << "The CBBA_path before path_remove is" <<std::endl;
	//	for (int i = 0; i < cbba_path.size();i++)
	//		std::cout << cbba_path[i] << ' ';
	//	std::cout << std::endl;
	//
	//	std::cout << "The cbba_bundle before path_remove is" <<std::endl;
	//	for (int i = 0; i < cbba_bundle.size();i++)
	//		std::cout << cbba_bundle[i] <<std::endl;
	//
		for (int j = 0; j < (*it_ag).cbba_path.size(); j++)
			for (int i = 0; i < (*it_ag).cbba_bundle.size(); i++)
				if ((*it_ag).cbba_path[j] == (*it_ag).cbba_bundle[i])
					existence[j] = 1;
	//
	//	std::cout << "The existence situation is " << std::endl;
	//	for (int i = 0; i < existence.size(); i++)
	//		std::cout << existence[i] << ' ';
	//	std::cout <<std::endl;

		// Remove all tasks whose existence is 0
		std::vector<float> cbba_path_origin;
		cbba_path_origin.clear();
		for (int i = 0; i < (*it_ag).cbba_path.size(); i++)
			cbba_path_origin.push_back((*it_ag).cbba_path[i]);

		for (int z = 0; z < existence.size(); z++)
			if (existence[z] == 0)
				(*it_ag).cbba_path.erase(std::remove((*it_ag).cbba_path.begin(),(*it_ag).cbba_path.end(),cbba_path_origin[z]),(*it_ag).cbba_path.end());

	//	std::cout << "The cbba_path after path_remove is" << std::endl;
	//	for (int i = 0; i < cbba_path.size();i++)
	//		std::cout << cbba_path[i] <<std::endl;
	}
}

void CBBA_sampling::bundle_add_for_sampling(){
	std::cout << "bundle adding ... " << std::endl;

	for (auto it_ag = all_agent_.begin(); it_ag != all_agent_.end(); it_ag++){
		bool bundleFull = -1;
		int desired_Index = -1;

		// The best position for all tasks which have not been added into the bundle
		// The index of the best_position is same as the index of task
		std::vector<int> best_position;

		//std::cout << "bundle add" << std::endl;
		if ((*it_ag).cbba_bundle.size() > max_bundle_length){
			std::cout << "The bundle is full" << std::endl;
			bundleFull = 1;
		}
		else
			bundleFull = 0;

		while(bundleFull == 0){

				// Print out the current bundle information
		//		std::cout << "CURRENT BUNDLE IS: " << Index << std::endl;
		//		for (int i = 0; i < cbba_bundle.size(); i++)
		//			std::cout << cbba_bundle[i] << ' ';
		//		std::cout << std::endl;

            //award_update;
            
            best_position = CBBA_sampling::award_update_for_sampling((*it_ag));
			// best_position = (*it_ag).award_update_for_sampling(Global_LTL_);
			// Update the awards mannually
			// std::cout << "The agent is" << Index <<std::endl;
			// award_update_manually();

            
			desired_Index = CBBA_sampling::desired_task_finder((*it_ag));
			//std::cout << "The desired task index is" << desired_Index << std::endl;
			if (desired_Index == -1)
				break;

			// Update the assignment
			(*it_ag).cbba_z[desired_Index] = (*it_ag).Index;
			(*it_ag).cbba_y[desired_Index] = (*it_ag).cbba_award[desired_Index];


			// Insert the desired task into the path with best insert position
			std::vector<int>::iterator it;
			it = (*it_ag).cbba_path.begin();
			(*it_ag).cbba_path.insert(it+best_position[desired_Index],desired_Index);

			// Insert the desired task into the bundle
			(*it_ag).cbba_bundle.push_back(desired_Index);

				// For test
		//		std::cout << "BUNDLE IS" << std::endl;
		//		for (int j = 0; j < cbba_bundle.size(); j++)
		//			std::cout << cbba_bundle[j] << ' ';
		//		std::cout <<std::endl;
		//
		//		std::cout << "PATH IS" << std::endl;
		//		for (int j = 0; j < cbba_path.size(); j++)
		//			std::cout << cbba_path[j] << ' ';
		//		std::cout <<std::endl;


			if ((*it_ag).cbba_bundle.size() > max_bundle_length)
				bundleFull = 1;
		}

		(*it_ag).history.z_history.push_back((*it_ag).cbba_z);
		(*it_ag).history.y_history.push_back((*it_ag).cbba_y);
	}

};


// Check whether assignment is over or not
bool CBBA_sampling::success_checker(){

	std::vector<int> BundlesForAllAgents;
	BundlesForAllAgents.clear();
	bool successFlag;
	successFlag = 0;
	int Num;
	int AppearOnce = 0;

	for (int i = 0; i < all_agent_.size(); i++)
		for (int j = 0; j < all_agent_[i].cbba_bundle.size(); j++)
			BundlesForAllAgents.push_back(all_agent_[i].cbba_bundle[j]);

	for (int j = 0; j < num_tasks_ ;j++){
		Num = std::count(BundlesForAllAgents.begin(),BundlesForAllAgents.end(),j);
		if (Num == 1)
			AppearOnce = AppearOnce + 1;
	}

	if (AppearOnce == num_tasks_)
		successFlag = 1;

	return successFlag;
};


std::vector<int> CBBA_sampling::award_update_for_sampling(cbba_Agent& agent){
	std::cout << "updating award for agent " << agent.Index << "..." << std::endl;
	
	// Calculate the length of path for current cbba_path
	// float length_path_origin = 0.0;

	// if (!cbba_path.empty()){
	// 	std::string ltl_new = LTLDecomposition::subtask_recreator(cbba_path,Global_LTL);
	// 	length_path_origin = CBBA::PathLengthCalculation(ltl_new, Index);
	// }
	// else
	// 	length_path_origin = 0.0;

	// Initialize the best position for all tasks
	std::vector<int> best_position;
	for (int j = 0; j < num_tasks_; j++)
		best_position.push_back(-1);

	std::vector<int> tasks_NotInBundle;
	// Initialize the tasks_NotInBundle
	tasks_NotInBundle.clear();

	// Put all the task index into the tasks_NotInBundle
	for(int j = 0; j < num_tasks_; j++)
		tasks_NotInBundle.push_back(j);

	// Find all tasks which have not been inserted into the bundle
	for (int j = 0; j < agent.cbba_bundle.size(); j++)
		// Erase all tasks which have been added into the bundle
		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),agent.cbba_bundle[j]),tasks_NotInBundle.end());

	// Initialize bundle_copy
	// There are cbba_bundle.size()+1 possible position where task can be inserted into
	std::vector<std::vector<int>> path_copy;

	// Find all possibilities each task in task_NotInBundle can insert into
	std::vector<int>::iterator it;
	std::vector<float> c_award;
	for (int j = 0; j < tasks_NotInBundle.size();j++){

		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
		// c_award and bundle_copy are only for one task with different insert position
		c_award.clear();
		path_copy.clear();

		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
		for (int k = 0; k < agent.cbba_path.size()+1; k++)
			path_copy.push_back(agent.cbba_path);

		// Insert one task (task_NotInBundle[j]) into all the possible position
		// Each item in the path_copy represents a possible position to insert the task
		for (int m = 0; m < path_copy.size(); m++){
			it = path_copy[m].begin();
			path_copy[m].insert(it+m,tasks_NotInBundle[j]);
		}

		// For test
		// check how many possibility we have
		//std::cout << "The bundle_copy size is:" << path_copy.size() << std::endl;
//		for (int i = 0; i < bundle_copy.size(); i++)
//			for (int m = 0; m < bundle_copy[i].size();m++)
//				std::cout << "The current bundle_copy is" << bundle_copy[i][m];
//			std::cout << std::endl;

		// Calculate the Astar length for each possible insert position
		// std::cout << "============debug==============" << std::endl;
		for (int i = 0; i < path_copy.size(); i++){
            std::string ltl_updated = LTLDecomposition::subtask_recreator(path_copy[i], Global_LTL_);

            // std::cout << "~========================DEBUG before=============================" << std::endl;
            c_award.push_back(CBBA_sampling::path_length_calculation(ltl_updated, agent));
            // std::cout << "~========================DEBUG after=============================" << std::endl;
		}
        
		float c_award_min = 1000;
		for (int i = 0; i < c_award.size(); i++)
			if (c_award[i] <= c_award_min){
				c_award_min = c_award[i];
				best_position[tasks_NotInBundle[j]] = i;
			}

		// Update the award
		agent.cbba_award[tasks_NotInBundle[j]] = 1000 - c_award_min;
		//std::cout << "c_award_min is" << c_award_min << std::endl;
		//std::cout << "tasks_NotInBundle[j] is " << tasks_NotInBundle[j] << std::endl;
	}

	// For test
//	std::cout << "The Awards are !!!!!!!!!!!!!!!!!!!!" << std::endl;
//	for (int j = 0; j < M; j++)
//		std::cout << cbba_award[j] << ' ';
//	std::cout << '\n';
//
//	std::cout << "Best Position for each task should be:" <<std::endl;
//	for (int j = 0; j < best_position.size(); j++)
//		std::cout << best_position[j] << ' ';
//	std::cout << std::endl;

	return best_position;
};


void CBBA_sampling::start_cbba(){
    bool succFlag = 0;
	// For test:
	//for(int It = 0; It < 3; It++){
	while (succFlag != 1){
		/*** 6. Communication among neighbors ***/
		communicate();
		//Update the history of iteration neighbors
		for (int i = 0; i < num_agent_; i++)
        all_agent_[i].history.iter_neighbors_his.push_back(all_agent_[i].iteration_neighbors);

		/*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		bundle_remove();
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		bundle_add_for_sampling();
		/*** 7.3 Check whether the assignment converge or not ***/
		succFlag = success_checker();
		std::cout << "The Flag for success is " << succFlag <<std::endl;
		/*** 7.4 Update the number of interation ***/
		// Increase the iteration
		for (int i = 0; i < num_agent_; i++){
            all_agent_[i].Iter++;
        }
            
    }
    for (int j = 0; j < all_agent_.size(); j++){
        std::cout << "agent " << j << ": ";
        for (int i= 0; i < all_agent_[j].cbba_path.size(); i++ ){
            std::cout << ", "<< all_agent_[j].cbba_path[i];
        }
        std::cout << std::endl;
    }
}

void CBBA_sampling::get_solution(){
    std::cout << "===========================================================" << std::endl;
    std::cout << "====================Getting solution ...====================" << std::endl;
    std::cout << "===========================================================" << std::endl;
    for (int i = 0; i < all_agent_.size(); i++) {
        cbba_Agent agent = all_agent_[i];
        std::vector<double> init_state = agent.init_state_;;
        LTL_SamplingDubins ltl_sampling_dubins;
        std::string ltl_new = LTLDecomposition::subtask_recreator(agent.cbba_path, Global_LTL_);

        std::cout << "For agent " << agent.Index << ", local LTL is: " << ltl_new << std::endl;

        // std::vector<int> indep_set;
        // std::vector<std::string> buchi_regions_indep = LTLDecomposition::ObtainBuchiRegion({ltl_new}).front();
        // std::vector<std::string> indep_set_str = buchi_regions_indep;
        // for (int j = 0; j < indep_set_str.size(); j++) {
        //     indep_set_str[j].erase(indep_set_str[j].begin());
        //     indep_set.push_back(std::stoi(indep_set_str[j]));
        // }
        ltl_sampling_dubins.read_formula(ltl_new, buchi_regions_, indep_set_);
        ltl_sampling_dubins.init_workspace(work_space_size_x_, work_space_size_y_);
        ltl_sampling_dubins.init_parameter(EPSILON_, RADIUS_, agent.radius_L_, agent.radius_R_);
        for (int i = 0; i < all_interest_regions_.size(); i++) {
            std::pair <double, double> position_x = all_interest_regions_[i].get_x_position();
            std::pair <double, double> position_y = all_interest_regions_[i].get_y_position();
            ltl_sampling_dubins.set_interest_region(position_x, position_y, all_interest_regions_[i].get_region_interest());
        }
    
        for (int i = 0; i < all_obstacles_.size(); i++) {
            std::pair <double, double> position_x = all_obstacles_[i].get_x_position();
            std::pair <double, double> position_y = all_obstacles_[i].get_y_position();
            ltl_sampling_dubins.set_obstacle(position_x, position_y);
        }
    
        ltl_sampling_dubins.set_init_state(init_state);
        ltl_sampling_dubins.start_sampling(iteration_cbba_*2);

        std::vector<std::vector<double>> path = ltl_sampling_dubins.get_path();
        all_agent_[i].traj_ = path;

        /// Vis
        lcm::LCM lcm;
        sampling::path_data path_data_;
        path_data_.num_state = path.size();
        path_data_.state_x.resize(path_data_.num_state);
        path_data_.state_y.resize(path_data_.num_state);
        for (int i = 0; i < path.size(); i++) {
            path_data_.state_x[i] = path[i][0];
            path_data_.state_y[i] = path[i][1];
        }
        lcm.publish("PATH", &path_data_);
    }
}