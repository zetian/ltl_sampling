
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

using namespace acel;

int main(int argc, char** argv )
{
	srand(time(NULL));
	lcm::LCM lcm;
	
	double EPSILON = 5;
    double RADIUS = 10;
    double radius_L = 15;
	double radius_R = 15;
	int iteration_cbba = 300;
	double work_space_size_x = 100;
	double work_space_size_y = 100;
	
    sampling::workspace_size_data space_data;
    space_data.size_x = work_space_size_x;
    space_data.size_y = work_space_size_y;
    lcm.publish("WORKSPACE", &space_data);
	/************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/

	LTLFormula Global_LTL;
	// Define the position of Target (p2,p3,p4,...)
	
	// Decompose the global LTL_expression
    // std::string safty = "([]!p0)";
    std::string safty = "";
	std::string liveness = "(<>p2) && (<>p0 && (<>p1))";
	// std::string liveness = "(<>p0) && (<>p1) && (<>p2)";
	LTLDecomposition::get_safety_properties(Global_LTL, safty);
	LTLDecomposition::get_liveness_properties(Global_LTL, liveness);
	
	Global_LTL = LTLDecomposition::GlobalLTLDecomposition(Global_LTL);

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
	std::vector<cbba_Agent> all_agent = {cbba_Agent(0,{1,1},y_0,z_0,y_his_0,z_his_0), cbba_Agent(1,{1,1},y_0,z_0,y_his_0,z_his_0)};

    sampling::sample_data node_data;
	all_agent[0].init_state_ = {20, 10, M_PI/2};
	all_agent[0].radius_L_ = 15;
    all_agent[0].radius_R_ = 15;
    node_data.state[0] = all_agent[0].init_state_[0];
    node_data.state[1] = all_agent[0].init_state_[1];
    lcm.publish("SAMPLE", &node_data);

	all_agent[1].init_state_ = {80, 10, M_PI/2};
	all_agent[1].radius_L_ = 15;
    all_agent[1].radius_R_ = 15;
    node_data.state[0] = all_agent[1].init_state_[0];
    node_data.state[1] = all_agent[1].init_state_[1];
    lcm.publish("SAMPLE", &node_data);

	int num_tasks = Global_LTL.Num_Tasks;
	// Initialize the awards
	for (int i = 0; i < all_agent.size(); i++){
		for (int j = 0; j < num_tasks; j++){
			all_agent[i].cbba_award.push_back(-1);
		}
	}


    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    buchi_regions.push_back("p1");
    buchi_regions.push_back("p2");
    std::vector<int> indep_set = {0, 1, 2};

	// LTL_SamplingDubins ltl_sampling_dubins;
	CBBA_sampling cbba_sampling;

	for (int i = 0; i < all_agent.size(); i++) {
		cbba_sampling.add_agent(all_agent[i]);
	}

	cbba_sampling.set_global_ltl(Global_LTL);
    cbba_sampling.set_buchi_regions(buchi_regions);
    cbba_sampling.set_indep_set(indep_set);
	cbba_sampling.init_workspace(work_space_size_x, work_space_size_y);
    cbba_sampling.init_parameter(iteration_cbba, EPSILON, RADIUS);

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
    cbba_sampling.get_solution();
    
    sampling::sample_draw draw;
    draw.if_draw = true;
    lcm.publish("DRAW_SAMPLE", &draw);

	return 0;

}


