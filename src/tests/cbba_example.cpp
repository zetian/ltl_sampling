/*
 * cbba_example.cpp
 *
 *  Created on: Oct 4, 2017
 *      Author: jfang
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

#include "trans_sys/cbba_Agent_sampling.h"
#include "trans_sys/ltl_formula_sampling.h"

using namespace cv;
using namespace acel;

int main(int argc, char** argv )
{

	/************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/

	LTLFormula Global_LTL;
	// Define the position of Target (p2,p3,p4,...)
	// Global_LTL.task_info = {{"p2",2},{"p3",3},{"p4",4},
			// {"p5",5},{"p6",6},{"p7",7},{"p8",8}};
			
	Global_LTL.task_info = {{"p0",0},{"p1",1},{"p2",2}};
	
	// Decompose the global LTL_expression
	std::string safty = "([]p0)";
	std::string liveness = "(<>p1) && (<>p2)";
	LTLDecomposition::get_safety_properties(Global_LTL, safty);
	LTLDecomposition::get_liveness_properties(Global_LTL, liveness);
	
	Global_LTL = LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	
	// std::cout <<"!!!!~~~~~~" << Global_LTL.LTL_expression.sub_LTL_expression[0] << std::endl;
	auto buchi_region = LTLDecomposition::ObtainBuchiRegion({safty + liveness});
	// std::cout <<"~~~~~" << buchi_region[0][0] << buchi_region[0][1] <<  buchi_region[0][2] << std::endl;
	std::vector<int> indep_set;
    // std::vector<std::string> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_new}).front();
    std::vector<std::string> indep_set_str = buchi_region.front();
    for (int i = 0; i < indep_set_str.size(); i++) {
		indep_set_str[i].erase(indep_set_str[i].begin());
		indep_set.push_back(std::stoi(indep_set_str[i]));
    }
	std::cout <<"~~~lalal~~" << indep_set[0] << indep_set[1] <<  indep_set[2] << std::endl;
	// std::vector<std::vector<std::string>> ObtainBuchiRegion(std::vector<std::string> expressions);

	// std::cout << "~~~~" << std::endl;
	//std::string ltl_formula_up = LTLDecomposition::subtask_recreator(bundle,Global_LTL);



	/*** 4. Initialize agents ***/
	std::vector<float> y_0;
	std::vector<int> z_0;
	for (int i = 0; i < Global_LTL.Num_Tasks; i++){
		y_0.push_back(0);
		z_0.push_back(-1);
	}
	std::vector<std::vector<float>> y_his_0 = {y_0};
	std::vector<std::vector<int>> z_his_0 = {z_0};

	// std::vector<cbba_Agent> agent = {cbba_Agent(0,15,{1,0,1,0},y_0,z_0,y_his_0,z_his_0),
	// 		cbba_Agent(1,14,{0,1,0,1},y_0,z_0,y_his_0,z_his_0),
	// 	    cbba_Agent(2,210,{1,0,1,1},y_0,z_0,y_his_0,z_his_0),
	// 		cbba_Agent(3,209,{0,1,1,1},y_0,z_0,y_his_0,z_his_0)};
	std::vector<cbba_Agent> agent = {cbba_Agent(0,{1,0,1,0},y_0,z_0,y_his_0,z_his_0),
			cbba_Agent(1,{0,1,0,1},y_0,z_0,y_his_0,z_his_0),
		    cbba_Agent(2,{1,0,1,1},y_0,z_0,y_his_0,z_his_0),
		    cbba_Agent(3,{0,1,1,1},y_0,z_0,y_his_0,z_his_0)};

	// Initialize the awards
	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 6; j++)
	    	agent[i].cbba_award.push_back(-1);

	/************************************************************************************************************/
	/**************************************          CBBA         ***********************************************/
	/************************************************************************************************************/
	bool succFlag = 0;
	// For test:
	//for(int It = 0; It < 3; It++){
	// while (succFlag != 1){
	// 	/*** 6. Communication among neighbors ***/
	// 	CBBA::communicate(agent);
	// 	//Update the history of iteration neighbors
	// 	for (int i = 0; i < N; i++)
	// 		agent[i].history.iter_neighbors_his.push_back(agent[i].iteration_neighbors);

	// 	/*** 7. Bundle Operations ***/
	// 	/*** 7.1 Remove the out-bid task from the bundle ***/
	// 	CBBA::bundle_remove(agent);
	// 	/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
	// 	CBBA::bundle_add_for_sampling(agent, Global_LTL);
	// 	/*** 7.3 Check whether the assignment converge or not ***/
	// 	succFlag = CBBA::success_checker(agent);
	// 	std::cout << "The Flag for success is " << succFlag <<std::endl;
	// 	/*** 7.4 Update the number of interation ***/
	// 	// Increase the iteration
	// 	for (int i = 0; i < N; i++)
	// 		agent[i].Iter++;
	// }

	return 0;

}


