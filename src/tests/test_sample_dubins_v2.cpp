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

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"

using namespace acel;


int main()
{
    srand(time(NULL));
    lcm::LCM lcm;

    // All parameters
    double EPSILON = 5;
    double RADIUS = 10;
    double radius_L = 30;
    double radius_R = 30;

    // Read formula
    // std::string ltl_formula = "<> (p2 && <> (p1 && (<> p0)))";
    // std::string ltl_formula = "<> p0 && <> p1 && <> p2";
    std::string ltl_formula = "(<>p0)";
    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    // buchi_regions.push_back("p1");
    // buchi_regions.push_back("p2");
    // std::vector<int> indep_set = {0, 1, 2};
    std::vector<int> indep_set = {0};
    
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    // Initial state
    // std::vector<double> init_state = {99, 1, M_PI/4*3};
    std::vector<double> init_state = {5, 5, 0};
    // Set up the class
    LTL_SamplingDubins ltl_sampling_dubins;
    ltl_sampling_dubins.read_formula(ltl_formula, buchi_regions, indep_set);
    ltl_sampling_dubins.init_workspace(work_space_size_x, work_space_size_y);
    ltl_sampling_dubins.init_parameter(EPSILON, RADIUS, radius_L, radius_R);
    
    // Add region of interests
    std::pair <double, double> position_x (55, 70);
    std::pair <double, double> position_y (20, 35);
    ltl_sampling_dubins.set_interest_region(position_x, position_y, 0);
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    // position_x = std::make_pair(55, 95);
    // position_y = std::make_pair(55, 95);
    // ltl_sampling_dubins.set_interest_region(position_x, position_y, 1);
    // r_data.position_x[0] =  position_x.first;
    // r_data.position_x[1] =  position_x.second;
    // r_data.position_y[0] =  position_y.first;
    // r_data.position_y[1] =  position_y.second;
    // lcm.publish("REGION", &r_data);

    // position_x = std::make_pair(10, 20);
    // position_y = std::make_pair(80, 90);
    // ltl_sampling_dubins.set_interest_region(position_x, position_y, 2);
    // r_data.position_x[0] =  position_x.first;
    // r_data.position_x[1] =  position_x.second;
    // r_data.position_y[0] =  position_y.first;
    // r_data.position_y[1] =  position_y.second;
    // lcm.publish("REGION", &r_data);

    // Add obstacles
    position_x = std::make_pair(0, 70);
    position_y = std::make_pair(15, 20);
    ltl_sampling_dubins.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    // position_x = std::make_pair(15, 40);
    // position_y = std::make_pair(65, 70);
    // ltl_sampling_dubins.set_obstacle(position_x, position_y);
    // r_data.position_x[0] =  position_x.first;
    // r_data.position_x[1] =  position_x.second;
    // r_data.position_y[0] =  position_y.first;
    // r_data.position_y[1] =  position_y.second;
    // lcm.publish("OBSTACLE", &r_data);
    
    // Set the initial state of the vehicle
    ltl_sampling_dubins.set_init_state(init_state);

    // Set the number of iterations
    int iterations = 1500;
    // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    // Start sampling searching
    ltl_sampling_dubins.start_sampling(iterations);

    std::vector<std::vector<double>> path = ltl_sampling_dubins.get_path();


    // Vis
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