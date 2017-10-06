
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv

#include "trans_sys/cbba_Agent_sampling.h"
#include "trans_sys/ltl_formula_sampling.h"
#include "sampling/cbba_sampling.h"

using namespace srcl;

int main(int argc, char** argv )
{
	srand(time(NULL));
	lcm::LCM lcm;
	
	double EPSILON = 5;
    double RADIUS = 10;
    double radius_L = 15;
	double radius_R = 15;
	
	double work_space_size_x = 100;
    double work_space_size_y = 100;
	/************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/

	LTLFormula Global_LTL;
	// Define the position of Target (p2,p3,p4,...)
	// Global_LTL.task_info = {{"p2",2},{"p3",3},{"p4",4},
			// {"p5",5},{"p6",6},{"p7",7},{"p8",8}};
			
	Global_LTL.task_info = {{"p1",0},{"p2",1}};
	
	// Decompose the global LTL_expression
	std::string safty = "([]!p0)";
	std::string liveness = "(<>p1) && (<>p2)";
	LTLDecomposition::get_safety_properties(Global_LTL, safty);
	LTLDecomposition::get_liveness_properties(Global_LTL, liveness);
	
	Global_LTL = LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	
	// std::cout <<"!!!!~~~~~~" << Global_LTL.LTL_expression.sub_LTL_expression[0] << std::endl;
	// auto buchi_region = LTLDecomposition::ObtainBuchiRegion({safty + liveness});
	// // std::cout <<"~~~~~" << buchi_region[0][0] << buchi_region[0][1] <<  buchi_region[0][2] << std::endl;
	// std::vector<int> indep_set;
    // // std::vector<std::string> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_new}).front();
    // std::vector<std::string> indep_set_str = buchi_region.front();
    // for (int i = 0; i < indep_set_str.size(); i++) {
	// 	indep_set_str[i].erase(indep_set_str[i].begin());
	// 	indep_set.push_back(std::stoi(indep_set_str[i]));
    // }
	// std::cout <<"~~~lalal~~" << indep_set[0] << indep_set[1] <<  indep_set[2] << std::endl;
	// std::vector<std::vector<std::string>> ObtainBuchiRegion(std::vector<std::string> expressions);

	// std::cout << "~~~~" << std::endl;
	//std::string ltl_formula_up = LTLDecomposition::subtask_recreator(bundle,Global_LTL);



	/*** 4. Initialize agents ***/
	std::vector<float> y_0;
    std::vector<int> z_0;
    std::cout << "Global_LTL.Num_Tasks: " <<Global_LTL.Num_Tasks <<std::endl;
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
	
	// std::vector<cbba_Agent> all_agent = {cbba_Agent(0,{1,0,1,0},y_0,z_0,y_his_0,z_his_0),
	// 		cbba_Agent(1,{0,1,0,1},y_0,z_0,y_his_0,z_his_0),
	// 	    cbba_Agent(2,{1,0,1,1},y_0,z_0,y_his_0,z_his_0),
	// 		cbba_Agent(3,{0,1,1,1},y_0,z_0,y_his_0,z_his_0)};
	std::vector<cbba_Agent> all_agent = {cbba_Agent(0,{1,1},y_0,z_0,y_his_0,z_his_0), cbba_Agent(1,{1,1},y_0,z_0,y_his_0,z_his_0)};
	all_agent[0].init_state_ = {20, 10, M_PI/2};
	all_agent[1].init_state_ = {80, 10, M_PI/2};

	int num_tasks = Global_LTL.task_info.size();
	// Initialize the awards
	for (int i = 0; i < all_agent.size(); i++){
		for (int j = 0; j < num_tasks; j++){
			all_agent[i].cbba_award.push_back(-1);
		}
	}
	    
			


	// LTL_SamplingDubins ltl_sampling_dubins;
	CBBA_sampling cbba_sampling;

	for (int i = 0; i < all_agent.size(); i++) {
		cbba_sampling.add_agent(all_agent[i]);
	}

	cbba_sampling.set_global_ltl(Global_LTL);

	cbba_sampling.init_workspace(work_space_size_x, work_space_size_y);
    cbba_sampling.init_parameter(EPSILON, RADIUS, radius_L, radius_R);

	// Add region of interests
    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    cbba_sampling.set_interest_region(position_x, position_y, 0);
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    cbba_sampling.set_interest_region(position_x, position_y, 1);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    cbba_sampling.set_interest_region(position_x, position_y, 2);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    // Add obstacles
    position_x = std::make_pair(35, 62);
    position_y = std::make_pair(35, 40);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(15, 40);
    position_y = std::make_pair(65, 70);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);


	cbba_sampling.start_cbba();



	/************************************************************************************************************/
	/**************************************          CBBA         ***********************************************/
	/************************************************************************************************************/
	// bool succFlag = 0;
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


