
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>

#include "trans_sys/cbba_Agent_sampling.h"
#include "trans_sys/ltl_formula_sampling.h"
#include "sampling/cbba_sampling.h"
#include "stopwatch/stopwatch.h"

using namespace acel;

int main(int argc, char** argv )
{
    // Random seed
    srand(time(NULL));
    // LCM for communication
    lcm::LCM lcm;
    // Timer
    stopwatch::StopWatch stopwatch;
    
    /*** Set up the seaching object ***/
    CBBA_sampling cbba_sampling;

    /*** Set the size of the workspace ***/
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    cbba_sampling.init_workspace(work_space_size_x, work_space_size_y);
    // Publish workspace size for visualization
    sampling::workspace_size_data space_data;
    space_data.size_x = work_space_size_x;
    space_data.size_y = work_space_size_y;
    lcm.publish("WORKSPACE", &space_data);

    /*** Set all parameters ***/
    // EPSILON is the forward step size when sampling searching
    double EPSILON = (work_space_size_x + work_space_size_y)/2/20;
    // RADIUS is the radius of checking aera when sampling searching
    double RADIUS = EPSILON*2;
    // radius_L is the left minimum turning radius
    double radius_L = 15;
    // radius_R is the right minimum turning radius
    double radius_R = 15;

    /*** Set the number of iterations ***/
    // Solution towards to optimal when iterations -> infinite
    int iteration_cbba = 500;
	cbba_sampling.init_parameter(iteration_cbba, EPSILON, RADIUS);

    /*** Read formula ***/
	LTLFormula Global_LTL;
    // std::string safty = "([]!p0)";
    std::string safty = "";
	// std::string liveness = "(<>p2) && (<>p0 && (<>p1))";
	std::string liveness = "(<>p0) && (<>p1) && (<>p2)";
	LTLDecomposition::get_safety_properties(Global_LTL, safty);
	LTLDecomposition::get_liveness_properties(Global_LTL, liveness);
	// Decompose the global LTL_expression
	Global_LTL = LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
    // Wrap all region of interests (ROI) as input for reading formula
    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    buchi_regions.push_back("p1");
    buchi_regions.push_back("p2");
    // indep_set store the ROI that independent to each other, in this case means p0, p1 and p2 have no intersections
    std::vector<int> indep_set = {0, 1, 2};
    cbba_sampling.set_global_ltl(Global_LTL);
    cbba_sampling.set_buchi_regions(buchi_regions);
    cbba_sampling.set_indep_set(indep_set);

	/*** Set the initial state and parameters of the UAVs ***/
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
    all_agent[0].ground_speed_ = 1;
    // Visual the init state
    node_data.state[0] = all_agent[0].init_state_[0];
    node_data.state[1] = all_agent[0].init_state_[1];
    lcm.publish("SAMPLE", &node_data);

	all_agent[1].init_state_ = {80, 10, M_PI/2};
	all_agent[1].radius_L_ = 15;
    all_agent[1].radius_R_ = 15;
    all_agent[0].ground_speed_ = 1;
    // Visual the init state
    node_data.state[0] = all_agent[1].init_state_[0];
    node_data.state[1] = all_agent[1].init_state_[1];
    lcm.publish("SAMPLE", &node_data);

	// Initialize the awards for cbba
	for (int i = 0; i < all_agent.size(); i++){
		for (int j = 0; j < Global_LTL.Num_Tasks; j++){
			all_agent[i].cbba_award.push_back(-1);
		}
	}
    // Add agents to searching object
	for (int i = 0; i < all_agent.size(); i++) {
		cbba_sampling.add_agent(all_agent[i]);
	}

	cbba_sampling.set_global_ltl(Global_LTL);
    cbba_sampling.set_buchi_regions(buchi_regions);
    cbba_sampling.set_indep_set(indep_set);

	/*** Set region of interests ***/
    // All ROI and obstacles are rectangle for now
    // Three parameters are x position, y position and the name of ROI (0 means p0)
    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    cbba_sampling.set_interest_region(position_x, position_y, 0);
    // For visualization
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    cbba_sampling.set_interest_region(position_x, position_y, 1);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    cbba_sampling.set_interest_region(position_x, position_y, 2);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    /*** Set obstacles ***/
    position_x = std::make_pair(35, 62);
    position_y = std::make_pair(35, 40);
    cbba_sampling.set_obstacle(position_x, position_y);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(15, 40);
    position_y = std::make_pair(65, 70);
    cbba_sampling.set_obstacle(position_x, position_y);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    /*** Start cbba and sampling searching ***/
    stopwatch.tic();
    cbba_sampling.start_cbba();
    std::cout << "Time used for searching: " << stopwatch.toc() << std::endl;

    /*** Get result ***/
    cbba_sampling.get_solution();
    
    // Visualize the result
    sampling::sample_draw draw;
    draw.if_draw = true;
    lcm.publish("DRAW_SAMPLE", &draw);

	return 0;

}


