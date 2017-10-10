
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
	
	double EPSILON = 8;
    double RADIUS = 16;
    double radius_L = 20;
	double radius_R = 20;
	int iteration_cbba = 800;
	double work_space_size_x = 150;
    double work_space_size_y = 150;
	/************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/

	LTLFormula Global_LTL;
	// Define the position of Target (p2,p3,p4,...)
	// Global_LTL.task_info = {{"p2",2},{"p3",3},{"p4",4},
			// {"p5",5},{"p6",6},{"p7",7},{"p8",8}};
			
	Global_LTL.task_info = {{"p0",0}, {"p1",1}, {"p2",2}, {"p3",3}, {"p4",4}, {"p5",5}, {"p6",6}, {"p7",7}};
	
	// Decompose the global LTL_expression
    // std::string safty = "([]!p0)";
    std::string safty = "";
    // std::string liveness = "(<>p0) && (<>p1) && (<>p2)";
    
    // std::string liveness = "(<>p0) && (<>p1) && (<>p2) && (<>p3) && (<>p4) && (<>p5 && (<>p6)) && (<>p7)";
    std::string liveness = "(<>p0) && (<>p1) && (<>p2) && (<>p3) && (<>p4) && (<>p5) && (<>p6) && (<>p7)";
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
	
	std::vector<cbba_Agent> all_agent = {cbba_Agent(0,{1,0,1,0},y_0,z_0,y_his_0,z_his_0),
			cbba_Agent(1,{0,1,0,1},y_0,z_0,y_his_0,z_his_0),
		    cbba_Agent(2,{1,0,1,1},y_0,z_0,y_his_0,z_his_0),
			cbba_Agent(3,{0,1,1,1},y_0,z_0,y_his_0,z_his_0)};
	// std::vector<cbba_Agent> all_agent = {cbba_Agent(0,{1,1},y_0,z_0,y_his_0,z_his_0), cbba_Agent(1,{1,1},y_0,z_0,y_his_0,z_his_0)};
    // sampling::sample_data node_data;
    all_agent[0].init_state_ = {10, 10, M_PI/2};
    all_agent[0].radius_L_ = radius_L;
    all_agent[0].radius_R_ = radius_R;
    // node_data.state[0] = all_agent[0].init_state_[0];
    // node_data.state[1] = all_agent[0].init_state_[1];
    // lcm.publish("SAMPLE", &node_data);
    
    all_agent[1].init_state_ = {140, 10, M_PI};
    all_agent[1].radius_L_ = radius_L;
    all_agent[1].radius_R_ = radius_R;
    // node_data.state[0] = all_agent[1].init_state_[0];
    // node_data.state[1] = all_agent[1].init_state_[1];
    // lcm.publish("SAMPLE", &node_data);

    all_agent[2].init_state_ = {10, 140, 0};
    all_agent[2].radius_L_ = radius_L;
    all_agent[2].radius_R_ = radius_R;
    // node_data.state[0] = all_agent[2].init_state_[0];
    // node_data.state[1] = all_agent[2].init_state_[1];
    // lcm.publish("SAMPLE", &node_data);

    all_agent[3].init_state_ = {140, 140, M_PI*3/2};
    all_agent[3].radius_L_ = radius_L;
    all_agent[3].radius_R_ = radius_R;
    // node_data.state[0] = all_agent[3].init_state_[0];
    // node_data.state[1] = all_agent[3].init_state_[1];
    // lcm.publish("SAMPLE", &node_data);


	int num_tasks = Global_LTL.task_info.size();
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
    buchi_regions.push_back("p3");
    buchi_regions.push_back("p4");
    buchi_regions.push_back("p5");
    buchi_regions.push_back("p6");
    buchi_regions.push_back("p7");
    std::vector<int> indep_set = {0, 1, 2, 3, 4, 5, 6, 7};

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
    std::pair <double, double> position_x (0, 20);
    std::pair <double, double> position_y (100, 120);
    cbba_sampling.set_interest_region(position_x, position_y, 0);
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(0, 20);
    position_y = std::make_pair(50, 70);
    cbba_sampling.set_interest_region(position_x, position_y, 1);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(40, 60);
    position_y = std::make_pair(20, 40);
    cbba_sampling.set_interest_region(position_x, position_y, 2);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(130, 150);
    position_y = std::make_pair(40, 60);
    cbba_sampling.set_interest_region(position_x, position_y, 3);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(90, 110);
    position_y = std::make_pair(30, 50);
    cbba_sampling.set_interest_region(position_x, position_y, 4);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(80, 100);
    position_y = std::make_pair(60, 80);
    cbba_sampling.set_interest_region(position_x, position_y, 5);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(40, 60);
    position_y = std::make_pair(70, 90);
    cbba_sampling.set_interest_region(position_x, position_y, 6);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(90, 110);
    position_y = std::make_pair(110, 130);
    cbba_sampling.set_interest_region(position_x, position_y, 7);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    // Add obstacles
    position_x = std::make_pair(0, 40);
    position_y = std::make_pair(70, 90);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(20, 40);
    position_y = std::make_pair(40, 70);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(0, 70);
    position_y = std::make_pair(120, 130);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(90, 150);
    position_y = std::make_pair(20, 30);
    cbba_sampling.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(110, 120);
    position_y = std::make_pair(80, 150);
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


