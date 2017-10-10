/*
 * cbba_Agent_sampling.cpp
 *
 *  Created on: Oct 4, 2017
 *      Author: jfang
 */

/*
 * cbba_Agent.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// #include "trans_sys/ltl_formula_sampling.h"
#include "trans_sys/cbba_Agent_sampling.h"

using namespace acel;

cbba_Agent::cbba_Agent(int id,std::vector<int> com,
		std::vector<float> y,std::vector<int> z,std::vector<std::vector<float>> y_his,std::vector<std::vector<int>> z_his)
{
	Index = id;
	// start_nodes_id_ = startID;
	communication = com;
	Iter = 0;
	cbba_bundle = {};
	cbba_path = {};
	iteration_neighbors = {0,0,0,0};
	history.iter_neighbors_his = {{0,0,0,0}};
	cbba_y = y;
	cbba_z = z;
	history.y_history = y_his;
	history.z_history = z_his;


};


// std::vector<int> cbba_Agent::award_update_for_sampling(LTLFormula Global_LTL){
// 	// Calculate the length of path for current cbba_path
// 	float length_path_origin = 0.0;

// 	// if (!cbba_path.empty()){
// 	// 	std::string ltl_new = LTLDecomposition::subtask_recreator(cbba_path,Global_LTL);
// 	// 	length_path_origin = CBBA::PathLengthCalculation(ltl_new, Index);
// 	// }
// 	// else
// 	// 	length_path_origin = 0.0;

// 	// Initialize the best position for all tasks
// 	std::vector<int> best_position;
// 	for (int j = 0; j < M; j++)
// 		best_position.push_back(-1);

// 	std::vector<int> tasks_NotInBundle;
// 	// Initialize the tasks_NotInBundle
// 	tasks_NotInBundle.clear();

// 	// Put all the task index into the tasks_NotInBundle
// 	for(int j = 0; j < M; j++)
// 		tasks_NotInBundle.push_back(j);

// 	// Find all tasks which have not been inserted into the bundle
// 	for (int j = 0; j < cbba_bundle.size(); j++)
// 		// Erase all tasks which have been added into the bundle
// 		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),cbba_bundle[j]),tasks_NotInBundle.end());

// 	// Initialize bundle_copy
// 	// There are cbba_bundle.size()+1 possible position where task can be inserted into
// 	std::vector<std::vector<int>> path_copy;

// 	// Find all possibilities each task in task_NotInBundle can insert into
// 	std::vector<int>::iterator it;
// 	std::vector<float> c_award;
// 	for (int j = 0; j < tasks_NotInBundle.size();j++){

// 		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
// 		// c_award and bundle_copy are only for one task with different insert position
// 		c_award.clear();
// 		path_copy.clear();

// 		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
// 		for (int k = 0; k < cbba_path.size()+1; k++)
// 			path_copy.push_back(cbba_path);

// 		// Insert one task (task_NotInBundle[j]) into all the possible position
// 		// Each item in the path_copy represents a possible position to insert the task
// 		for (int m = 0; m < path_copy.size(); m++){
// 			it = path_copy[m].begin();
// 			path_copy[m].insert(it+m,tasks_NotInBundle[j]);
// 		}

// 		// For test
// 		// check how many possibility we have
// 		//std::cout << "The bundle_copy size is:" << path_copy.size() << std::endl;
// //		for (int i = 0; i < bundle_copy.size(); i++)
// //			for (int m = 0; m < bundle_copy[i].size();m++)
// //				std::cout << "The current bundle_copy is" << bundle_copy[i][m];
// //			std::cout << std::endl;

// 		// Calculate the Astar length for each possible insert position
// 		for (int i = 0; i < path_copy.size(); i++){
// 			std::string ltl_updated = LTLDecomposition::subtask_recreator(path_copy[i],Global_LTL);
// 			c_award.push_back(CBBA::PathLengthCalculation(ltl_updated, Index));
// 		}

// 		float c_award_min = 100;
// 		for (int i = 0; i < c_award.size(); i++)
// 			if (c_award[i] <= c_award_min){
// 				c_award_min = c_award[i];
// 				best_position[tasks_NotInBundle[j]] = i;
// 			}

// 		// Update the award
// 		cbba_award[tasks_NotInBundle[j]] = 100-c_award_min;
// 		//std::cout << "c_award_min is" << c_award_min << std::endl;
// 		//std::cout << "tasks_NotInBundle[j] is " << tasks_NotInBundle[j] << std::endl;
// 	}

// 	// For test
// //	std::cout << "The Awards are !!!!!!!!!!!!!!!!!!!!" << std::endl;
// //	for (int j = 0; j < M; j++)
// //		std::cout << cbba_award[j] << ' ';
// //	std::cout << '\n';
// //
// //	std::cout << "Best Position for each task should be:" <<std::endl;
// //	for (int j = 0; j < best_position.size(); j++)
// //		std::cout << best_position[j] << ' ';
// //	std::cout << std::endl;

// 	return best_position;
// };

// Update the assignment from bundle information
void cbba_Agent::assignment_update(){
	if (!cbba_bundle.empty())
		for (int j = 0; j < cbba_bundle.size();j++)
			cbba_x[cbba_bundle[j]] = 1;
};


// float CBBA::PathLengthCalculation(std::string ltl_new, int agent_idx_){
// 	float path_length = 0.0;

// 	return path_length;
// };

// // Find the neighbors of agent and update neighbors
// void CBBA::neighbor_finder(std::vector<cbba_Agent>& agent)
// {
// 	for (auto it = agent.begin(); it != agent.end(); it++){
// 		(*it).neighbors.clear();
// 		for (int i = 0; i < agent.size(); i++)
// 			if ((*it).communication[i] == 1 && (*it).Index != i)
// 				(*it).neighbors.push_back(agent[i].Index);
// 			else
// 				continue;
// 	}

// //	std::cout << "Check out the neighbors" << std::endl;
// //	for (auto it1 = agent.begin(); it1 != agent.end(); it1++){
// //		for (auto it2 = (*it1).neighbors.begin(); it2 != (*it1).neighbors.end(); it2++)
// //			std::cout << (*it2) << " ";
// //	    std::cout << std::endl;
// //	}
// };

// // Let agent communicate with its neighbors
// void CBBA::communicate(std::vector<cbba_Agent>& agent)
// {
// 	CBBA::neighbor_finder(agent);
// 	// sender: itself k
// 	// receiver: i
// 	// task : j

// 	for (auto it_ag = agent.begin(); it_ag != agent.end(); it_ag++){
// 		for (int i = 0; i < (*it_ag).neighbors.size();i++){
// 			for (int j = 0; j < M; j++){
// 				// Entries 1 to 4
// 				// if current agent k thinks that the winner of task j is itself k
// 				if ((*it_ag).history.z_history.back()[j] == (*it_ag).Index){

// 					/***************************************** Entry 1 ***************************************/
// 					// Entry 1: Update or leave
// 					// If the receiver (neighbor) i thinks the winner of task j is also itself i
// 					if (agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).neighbors[i]){
// 						// Update
// 						if ((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j] > eps){
// 							//std::cout << "case 1" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 					    // Equal score: require to break the tie
// 						else if(abs((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
// 							// select the winner of task j as the agent with smaller index
// 							if (agent[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
// 								//std::cout << "case 2" << std::endl;
// 								agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 								agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 							}
// 						}
// 					}

// 					/***************************************** Entry 2 ***************************************/
// 					// Entry 2: Update
// 					// If the receiver i thinks the winner of task j is also agent k (sender)
// 					// Update
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
// 						//std::cout << "case 3" << std::endl;
// 						agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 						agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 					}

// 					/***************************************** Entry 3 ***************************************/

// 					// Entry 3: Update or Leave
// 					// If the receiver i thinks the winner of task j is not k or itself i but other agent
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
// 						// Compare the iteration of task j, find which one is the least information of task j
// 						// Update
// 						if ((*it_ag).history.iter_neighbors_his.back()[agent[(*it_ag).neighbors[i]].cbba_z[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[agent[(*it_ag).neighbors[i]].cbba_z[j]]){
// 							//std::cout << "case 4" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 						// Update
// 						else if((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j] > eps){
// 							//std::cout << "case 5" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 						// Equal scores: break the tie by selecting the winner as the agent with smaller index
// 						else if ((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j] <= eps){
// 							if (agent[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
// 								//std::cout << "case 6" << std::endl;
// 								agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 								agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 							}
// 						}
// 					}

// 					/***************************************** Entry 4 ***************************************/
// 					// Entry 4: Update
// 					// If the agent i (receiver) has no idea about the winner
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == -1){
// 						//std::cout << "case 7" << std::endl;
// 						agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 						agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 					}

// 					else
// 						std::cout << "Unknown winner value 1" << std::endl;
// 				}



// 		        /*********************************************************************************************************/
// 				/*********************************************************************************************************/
// 				/*********************************************************************************************************/

// 				// Entries 5 to 8
// 				// If current agent i (sender) thinks the winner of task j is agent k (receiver)
// 				else if ((*it_ag).history.z_history.back()[j] == agent[(*it_ag).neighbors[i]].Index){

// 					/***************************************** Entry 5 ***************************************/
// 					// Entry 5
// 				    // if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
// 				    // Leave
// 				    if (agent[(*it_ag).neighbors[i]].cbba_z[j] == agent[(*it_ag).neighbors[i]].Index)
// 				    	std::cout << "Do nothing Entry 5" << std::endl;

// 				    /***************************************** Entry 6 ***************************************/
// 				    // Entry 6
// 					// If agent i (receiver) thinks the winner of task j is agent k (sender)
// 					// Reset (Because the agent k will definitely be the first one to know its own information )
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
// 						//std::cout << "case 7" << std::endl;
// 						agent[(*it_ag).neighbors[i]].cbba_z[j] = -1;
// 						agent[(*it_ag).neighbors[i]].cbba_y[j] = -1;
// 					}

// 				    /***************************************** Entry 7 ***************************************/
// 				    // Entry 7
// 					// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
// 						// Compare the iteration of agent k and i, find which one has the least information
// 						// Reset
// 						if ((*it_ag).history.iter_neighbors_his.back()[agent[(*it_ag).neighbors[i]].cbba_z[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[agent[(*it_ag).neighbors[i]].cbba_z[j]]){
// 							// agent k should have the updated information
// 							//std::cout << "case 8" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = -1;
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = -1;
// 						}
// 					}

// 				    /***************************************** Entry 8 ***************************************/
// 				    // Entry 8
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == -1)
// 						std::cout <<"Do nothing Entry 8" << std::endl;

// 					else
// 						std::cout << "Unknown winner value 2" << std::endl;

// 				}

// 				/*********************************************************************************************************/
// 				/*********************************************************************************************************/
// 				/*********************************************************************************************************/
// 				// Entries 9 to 13
// 				// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
// 				else if ((*it_ag).history.z_history.back()[j] >= 0){
// 					/***************************************** Entry 9 ***************************************/
// 					// Entry 9
// 					// if agent i (receiver) thinks the winner of task j should be itself i
// 					if (agent[(*it_ag).neighbors[i]].cbba_z[j] == agent[(*it_ag).neighbors[i]].Index){
// 						// compare the iteration that agent k and i talk to the winner that agent k thinks
// 						// If agent k (sender) has the least information
// 						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 							// Update
// 							if ((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j] > eps){
// 								//std::cout << "case 8" << std::endl;
// 								agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 								agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 							}
// 							// If we have a tie: break the tie by selecting the agent with smaller index as the winner
// 							else if (abs((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
// 								if (agent[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
// 									// Update
// 									//std::cout << "case 9" << std::endl;
// 									agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 									agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 								}
// 							}
// 						}
// 					}

// 					/***************************************** Entry 10 ***************************************/
// 					// Entry 10
// 					// if agent i (receiver) thinks the winner of task j is agent k (sender)
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
// 						// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
// 						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 							//std::cout << "case 10" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 						else{
// 							// Reset
// 							//std::cout << "case 11" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = -1;
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = -1;
// 						}
// 					}

// 					/***************************************** Entry 11 ***************************************/
// 					// Entry 11
// 					// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).history.z_history.back()[j]){
// 						// Update
// 						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 							//std::cout << "case 12" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 					}

// 					/***************************************** Entry 12 ***************************************/
// 					// Entry 12
// 					// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
// 						if ((*it_ag).history.iter_neighbors_his.back()[agent[(*it_ag).neighbors[i]].cbba_z[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[agent[(*it_ag).neighbors[i]].cbba_z[j]]){
// 							// Update
// 							if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] >= agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 								//std::cout << "case 13" << std::endl;
// 								agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 								agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 							}
// 							// Reset
// 							else if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] < agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 								//std::cout << "case 14" << std::endl;
// 								agent[(*it_ag).neighbors[i]].cbba_z[j] = -1;
// 								agent[(*it_ag).neighbors[i]].cbba_y[j] = -1;
// 							}
// 							else
// 								std::cout << "Should not be here Entry 12" << std::endl;
// 						}
// 						else{
// 							if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 								// Update
// 								if ((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j] > eps){
// 									//std::cout << "case 15" << std::endl;
// 									agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 									agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 								}
// 								// If we have tie, break the tie by selecting the agent as the winner with smaller index
// 								else if (abs((*it_ag).history.y_history.back()[j] - agent[(*it_ag).neighbors[i]].cbba_y[j]) <= eps){
// 									if (agent[(*it_ag).neighbors[i]].cbba_z[j] > (*it_ag).history.z_history.back()[j]){
// 										//std::cout << "case 16" << std::endl;
// 										agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 										agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 									}
// 								}
// 							}
// 						}
// 					}
// 					/***************************************** Entry 13 ***************************************/
// 					// Entry 13
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == -1){
// 						// Update
// 						if ((*it_ag).history.iter_neighbors_his.back()[(*it_ag).history.z_history.back()[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).history.z_history.back()[j]]){
// 							//std::cout << "case 17" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 					}
// 					else
// 						std::cout << "Unknown winner value Entry 13" << std::endl;

// 				}

// 				/*********************************************************************************************************/
// 				/*********************************************************************************************************/
// 				/*********************************************************************************************************/
// 				// Entries 14 to 17
// 				else if ((*it_ag).history.z_history.back()[j] == -1){

// 					/***************************************** Entry 14 ***************************************/
// 					// Entry 14
// 					// Leave
// 					if (agent[(*it_ag).neighbors[i]].cbba_z[j] == agent[(*it_ag).neighbors[i]].Index)
// 						std::cout << "Do nothing Entry 14" << std::endl;

// 					/***************************************** Entry 15 ***************************************/
// 					// Entry 15
// 					// Update
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] == (*it_ag).Index){
// 						//std::cout << "case 18" << std::endl;
// 						agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 						agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 					}

// 					/***************************************** Entry 16 ***************************************/
// 					// Entry 16
// 					// Update
// 					else if (agent[(*it_ag).neighbors[i]].cbba_z[j] >= 0){
// 						// Update
// 						if ((*it_ag).history.iter_neighbors_his.back()[agent[(*it_ag).neighbors[i]].cbba_z[j]] > agent[(*it_ag).neighbors[i]].iteration_neighbors[agent[(*it_ag).neighbors[i]].cbba_z[j]]){
// 							//std::cout << "case 19" << std::endl;
// 							agent[(*it_ag).neighbors[i]].cbba_z[j] = (*it_ag).history.z_history.back()[j];
// 							agent[(*it_ag).neighbors[i]].cbba_y[j] = (*it_ag).history.y_history.back()[j];
// 						}
// 					}

// 					/***************************************** Entry 17 ***************************************/
// 					// Entry 17
// 					// Leave
// 					else if(agent[(*it_ag).neighbors[i]].cbba_z[j] == -1)
// 						std::cout<< "Do noting Entry 17" << std::endl;

// 					else
// 						std::cout << "Unknown winner value Entry 17" <<std::endl;
// 				}

// 				else
// 					std::cout << "Unknown winner value end of communicate" <<std::endl;
// 			}

// 			for (int n = 0; n < N; n++){
// 				if (n != (*it_ag).neighbors[i] && agent[(*it_ag).neighbors[i]].iteration_neighbors[n] < (*it_ag).history.iter_neighbors_his.back()[n]){
// 					agent[(*it_ag).neighbors[i]].iteration_neighbors[n] = (*it_ag).history.iter_neighbors_his.back()[n];
// 				}
// 			}

// 			agent[(*it_ag).neighbors[i]].iteration_neighbors[(*it_ag).Index] = (*it_ag).Iter;
// 		}

// 	}
// };

// // Find the available tasks for agent
// void CBBA::available_tasks_finder(cbba_Agent& agent_sig){
// 	//Initialize the available tasks
// 	agent_sig.h_avai.clear();
// 	bool condition_1;
// 	bool condition_2;

// 	for (int j = 0; j < M; j++){

// 		// Initialize the condition for each task
// 		condition_1 = 0;
// 		condition_2 = 0;

// 		if(agent_sig.cbba_award[j] - agent_sig.cbba_y[j] > eps)
// 			condition_1 = 1;
// 		else if (abs(agent_sig.cbba_award[j] - agent_sig.cbba_y[j]) <= eps)
// 			if (agent_sig.Index < agent_sig.cbba_z[j])
// 				condition_2 = 1;

// 		if (condition_1 == 1 || condition_2 == 1)
// 			agent_sig.h_avai.push_back(j);
// 	}

// //		std::cout << "The available task is" <<std::endl;
// //		for (int j = 0; j < h_avai.size();j++)
// //			std::cout << h_avai[j] << ' ';
// //		std::cout << std::endl;
// };


// // Find the desired task for agent
// int CBBA::desired_task_finder(cbba_Agent& agent_sig){
// 	int max = 0;
// 	int desired_Index = -1;

// 	// Update the available tasks for agent
// 	//std::cout << "P1" <<std::endl;
// 	CBBA::available_tasks_finder(agent_sig);

// 	if (!agent_sig.h_avai.empty()){
// 		// If for certain agent, there are more than one desired tasks, select the one with smaller task index
// 		for (int j = 0; j < agent_sig.h_avai.size();j++)
// 			if (agent_sig.cbba_award[agent_sig.h_avai[j]] > max){
// 		    	max = agent_sig.cbba_award[agent_sig.h_avai[j]];
// 		    	desired_Index = agent_sig.h_avai[j];
// 			}
// 	}

// 	return desired_Index;
// };


// // Remove the task from bundle and all the tasks added after it
// void CBBA::bundle_remove(std::vector<cbba_Agent>& agent){
// 	//std::cout << "bundle remove" << std::endl;

// 	for (auto it_ag = agent.begin(); it_ag != agent.end(); it_ag++){
// 		bool outbidForTask = 0;
// 		if (!(*it_ag).cbba_bundle.empty()){
// 			// Check whether agent is outbid by other agent for the tasks which are in the bundle
// 			for (int k = 0; k < (*it_ag).cbba_bundle.size(); k++){
// 				if ((*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] != (*it_ag).Index)
// 					outbidForTask = 1;

// 				// Remove the tasks added after the outbid task
// 				if (outbidForTask == 1){
// 					if ((*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] == (*it_ag).Index){
// 						(*it_ag).cbba_z[(*it_ag).cbba_bundle[k]] = -1;
// 						(*it_ag).cbba_y[(*it_ag).cbba_bundle[k]] = -1;
// 					}

// 					// Once the path is taken into consideration
// 				    // we need to find the position of task in the path

// 				    // remove the task from the bundle
// 					(*it_ag).cbba_bundle[k] = -1;
// 				}
// 			}
// 		}

// 		// Remove the task which has -1
// 		(*it_ag).cbba_bundle.erase(std::remove((*it_ag).cbba_bundle.begin(),(*it_ag).cbba_bundle.end(),-1),(*it_ag).cbba_bundle.end());


// 		std::cout << "The cbba_bunlde after bundle_remove is " << std::endl;
// 		for (int i = 0; i < (*it_ag).cbba_bundle.size();i++)
// 			std::cout << (*it_ag).cbba_bundle[i] <<std::endl;

// 		// Remove the tasks which are not showed in the bundle
// 		//agent = path_remove(std::vector<cbba_Agent> agent);
// 	}

// 	CBBA::path_remove(agent);
// };

// void CBBA::path_remove(std::vector<cbba_Agent>& agent){

// 	for(auto it_ag = agent.begin(); it_ag != agent.end(); it_ag++){
// 		std::vector<bool> existence;
// 		existence.clear();
// 		for (int m = 0; m < (*it_ag).cbba_path.size(); m++)
// 			existence.push_back(0);

// 		// For test
// 	//	std::cout << "The CBBA_path before path_remove is" <<std::endl;
// 	//	for (int i = 0; i < cbba_path.size();i++)
// 	//		std::cout << cbba_path[i] << ' ';
// 	//	std::cout << std::endl;
// 	//
// 	//	std::cout << "The cbba_bundle before path_remove is" <<std::endl;
// 	//	for (int i = 0; i < cbba_bundle.size();i++)
// 	//		std::cout << cbba_bundle[i] <<std::endl;
// 	//
// 		for (int j = 0; j < (*it_ag).cbba_path.size(); j++)
// 			for (int i = 0; i < (*it_ag).cbba_bundle.size(); i++)
// 				if ((*it_ag).cbba_path[j] == (*it_ag).cbba_bundle[i])
// 					existence[j] = 1;
// 	//
// 	//	std::cout << "The existence situation is " << std::endl;
// 	//	for (int i = 0; i < existence.size(); i++)
// 	//		std::cout << existence[i] << ' ';
// 	//	std::cout <<std::endl;

// 		// Remove all tasks whose existence is 0
// 		std::vector<float> cbba_path_origin;
// 		cbba_path_origin.clear();
// 		for (int i = 0; i < (*it_ag).cbba_path.size(); i++)
// 			cbba_path_origin.push_back((*it_ag).cbba_path[i]);

// 		for (int z = 0; z < existence.size(); z++)
// 			if (existence[z] == 0)
// 				(*it_ag).cbba_path.erase(std::remove((*it_ag).cbba_path.begin(),(*it_ag).cbba_path.end(),cbba_path_origin[z]),(*it_ag).cbba_path.end());

// 	//	std::cout << "The cbba_path after path_remove is" << std::endl;
// 	//	for (int i = 0; i < cbba_path.size();i++)
// 	//		std::cout << cbba_path[i] <<std::endl;
// 	}
// }

// void CBBA::bundle_add_for_sampling(std::vector<cbba_Agent>& agent, LTLFormula Global_LTL){

// 	for (auto it_ag = agent.begin(); it_ag != agent.end(); it_ag++){
// 		bool bundleFull = -1;
// 		int desired_Index = -1;

// 		// The best position for all tasks which have not been added into the bundle
// 		// The index of the best_position is same as the index of task
// 		std::vector<int> best_position;

// 		//std::cout << "bundle add" << std::endl;
// 		if ((*it_ag).cbba_bundle.size() > max_bundle_length){
// 			std::cout << "The bundle is full" << std::endl;
// 			bundleFull = 1;
// 		}
// 		else
// 			bundleFull = 0;

// 		while(bundleFull == 0){

// 				// Print out the current bundle information
// 		//		std::cout << "CURRENT BUNDLE IS: " << Index << std::endl;
// 		//		for (int i = 0; i < cbba_bundle.size(); i++)
// 		//			std::cout << cbba_bundle[i] << ' ';
// 		//		std::cout << std::endl;

// 			//award_update;
// 			best_position = (*it_ag).award_update_for_sampling(Global_LTL);
// 			// Update the awards mannually
// 			// std::cout << "The agent is" << Index <<std::endl;
// 			// award_update_manually();


// 			desired_Index = CBBA::desired_task_finder((*it_ag));
// 			//std::cout << "The desired task index is" << desired_Index << std::endl;
// 			if (desired_Index == -1)
// 				break;

// 			// Update the assignment
// 			(*it_ag).cbba_z[desired_Index] = (*it_ag).Index;
// 			(*it_ag).cbba_y[desired_Index] = (*it_ag).cbba_award[desired_Index];


// 			// Insert the desired task into the path with best insert position
// 			std::vector<int>::iterator it;
// 			it = (*it_ag).cbba_path.begin();
// 			(*it_ag).cbba_path.insert(it+best_position[desired_Index],desired_Index);

// 			// Insert the desired task into the bundle
// 			(*it_ag).cbba_bundle.push_back(desired_Index);

// 				// For test
// 		//		std::cout << "BUNDLE IS" << std::endl;
// 		//		for (int j = 0; j < cbba_bundle.size(); j++)
// 		//			std::cout << cbba_bundle[j] << ' ';
// 		//		std::cout <<std::endl;
// 		//
// 		//		std::cout << "PATH IS" << std::endl;
// 		//		for (int j = 0; j < cbba_path.size(); j++)
// 		//			std::cout << cbba_path[j] << ' ';
// 		//		std::cout <<std::endl;


// 			if ((*it_ag).cbba_bundle.size() > max_bundle_length)
// 				bundleFull = 1;
// 		}

// 		(*it_ag).history.z_history.push_back((*it_ag).cbba_z);
// 		(*it_ag).history.y_history.push_back((*it_ag).cbba_y);
// 	}

// };


// // Check whether assignment is over or not
// bool CBBA::success_checker(std::vector<cbba_Agent> agent){

// 	std::vector<int> BundlesForAllAgents;
// 	BundlesForAllAgents.clear();
// 	bool successFlag;
// 	successFlag = 0;
// 	int Num;
// 	int AppearOnce = 0;

// 	for (int i = 0; i < agent.size(); i++)
// 		for (int j = 0; j < agent[i].cbba_bundle.size(); j++)
// 			BundlesForAllAgents.push_back(agent[i].cbba_bundle[j]);

// 	for (int j = 0; j < M ;j++){
// 		Num = std::count(BundlesForAllAgents.begin(),BundlesForAllAgents.end(),j);
// 		if (Num == 1)
// 			AppearOnce = AppearOnce + 1;
// 	}

// 	if (AppearOnce == M)
// 		successFlag = 1;

// 	return successFlag;
// };
