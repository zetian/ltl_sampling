#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <bitset>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <random>

#include "trajectory/dubins_steer.h"
#include "trans_sys/spot_hoa_interpreter.h"
#include "sampling/ltl_sampling_dubins.h"
#include "stopwatch/stopwatch.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"

using namespace acel;

int main()
{
    // Random seed
    srand(time(NULL));
    // LCM for communication
    lcm::LCM lcm;
    // Timer
    stopwatch::StopWatch stopwatch;

    /*** Set up the seaching object ***/
    LTL_SamplingDubins ltl_sampling_dubins;

    /*** Set the size of the workspace ***/
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    ltl_sampling_dubins.init_workspace(work_space_size_x, work_space_size_y);
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
    ltl_sampling_dubins.init_parameter(EPSILON, RADIUS, radius_L, radius_R);

    /*** Read formula ***/
    // "(<> p0) && (<> p1) && (<> p2)" means visit p0, p1 and p2 region of interests
    std::string ltl_formula = "(<> p0) && (<> p1) && (<> p2)";
    // "<> (p0 && <> (p1 && (<> p2)))" means visit p0, p1 and p2 region of interests and by this order
    // std::string ltl_formula = "<> (p2 && <> (p1 && (<> p0)))";
    // Wrap all region of interests (ROI) as input for reading formula
    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    buchi_regions.push_back("p1");
    buchi_regions.push_back("p2");
    // indep_set store the ROI that independent to each other, in this case means p0, p1 and p2 have no intersections
    std::vector<int> indep_set = {0, 1, 2};
    ltl_sampling_dubins.read_formula(ltl_formula, buchi_regions, indep_set);

    /*** Set the initial state of the UAV ***/
    std::vector<double> init_state = {50, 10, M_PI/2};
    ltl_sampling_dubins.set_init_state(init_state);
    
    /*** Set region of interests ***/
    // All ROI and obstacles are rectangle for now
    // Three parameters are x position, y position and the name of ROI (0 means p0)
    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    ltl_sampling_dubins.set_interest_region(position_x, position_y, 0);
    // For visualization
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    ltl_sampling_dubins.set_interest_region(position_x, position_y, 1);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    ltl_sampling_dubins.set_interest_region(position_x, position_y, 2);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    /*** Set obstacles ***/
    position_x = std::make_pair(35, 62);
    position_y = std::make_pair(35, 40);
    ltl_sampling_dubins.set_obstacle(position_x, position_y);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(15, 40);
    position_y = std::make_pair(65, 70);
    ltl_sampling_dubins.set_obstacle(position_x, position_y);
    // For visualization
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);
    
    /*** Set the number of iterations ***/
    int iterations = 2000;

    /*** Start sampling searching ***/
    stopwatch.tic();
    ltl_sampling_dubins.start_sampling(iterations);
    std::cout << "Time used for searching: " << stopwatch.toc() << std::endl;

    /*** Get result ***/
    std::vector<std::vector<double>> path = ltl_sampling_dubins.get_path();

    // Visualize the result
    sampling::path_data path_data_;
    path_data_.num_state = path.size();
    path_data_.state_x.resize(path_data_.num_state);
    path_data_.state_y.resize(path_data_.num_state);
    for (int i = 0; i < path.size(); i++) {
        path_data_.state_x[i] = path[i][0];
        path_data_.state_y[i] = path[i][1];
    }
    lcm.publish("PATH", &path_data_);
    sampling::sample_draw draw;
    draw.if_draw = true;
    lcm.publish("DRAW_SAMPLE", &draw);

    

    return 0;
}