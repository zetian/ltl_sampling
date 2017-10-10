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

#include "sampling/ltl_sampling_simple.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"

using namespace acel;


int main()
{
    srand(time(NULL));
    lcm::LCM lcm;
    double EPSILON = 6;
    double RADIUS = 12;
    std::string ltl_formula = "(<> p0) && (<> p1) && (<> p2)";
    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    buchi_regions.push_back("p1");
    buchi_regions.push_back("p2");
    std::vector<int> indep_set = {0, 1, 2};
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    std::vector<double> init_state = {50, 10};

    LTL_SamplingSimple ltl_sampling_simple;
    ltl_sampling_simple.read_formula(ltl_formula, buchi_regions, indep_set);
    ltl_sampling_simple.init_workspace(work_space_size_x, work_space_size_y);


    // sampling::all_regions all_regions_;
    // all_regions_.num_region = 3;

    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    ltl_sampling_simple.set_interest_region(position_x, position_y, 0);
    sampling::region_data r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[0] = r_data;

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    ltl_sampling_simple.set_interest_region(position_x, position_y, 1);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[1] = r_data;

    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    ltl_sampling_simple.set_interest_region(position_x, position_y, 2);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[2] = r_data;
    position_x = std::make_pair(35, 62);
    position_y = std::make_pair(35, 40);
    ltl_sampling_simple.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(15, 40);
    position_y = std::make_pair(65, 70);
    ltl_sampling_simple.set_obstacle(position_x, position_y);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("OBSTACLE", &r_data);

    ltl_sampling_simple.set_init_state(init_state);
    int interation = 1000;

    ltl_sampling_simple.start_sampling(interation);
    std::vector<std::vector<double>> path = ltl_sampling_simple.get_path();

    sampling::path_data path_data_;
    path_data_.num_state = path.size();
    path_data_.state_x.resize(path_data_.num_state);
    path_data_.state_y.resize(path_data_.num_state);
    for (int i = 0; i < path.size(); i++) {
        
        path_data_.state_x[i] = path[i][0];
        
        path_data_.state_y[i] = path[i][1];
    }
    std::cout << "Length of the solution path: " << path.size() << std::endl;
    lcm.publish("PATH", &path_data_);
    sampling::sample_draw draw;
    draw.if_draw = true;
    // lcm.publish("DRAW_REGION", &draw);
    lcm.publish("DRAW_SAMPLE", &draw);



    
    return 0;
}