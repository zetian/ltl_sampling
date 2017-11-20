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
#include "multi_sampling/multi_sampling_dubins.h"
#include "stopwatch/stopwatch.h"
#include "config_reader/config_reader.h"

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
    // Read the configuration file
    std::string filename;
    // std::cout << "This program will let the user input a configuration filename with its path. e.g. ../../config/test_sample_dubins.ini" << std::endl;
    // std::cout << "\nPlease enter your desired filename path." << std::endl;
    // std::cin >> filename;
    ConfigReader config_reader("../../config/test_multi_dubins.ini");
    // ConfigReader config_reader(filename);
    if (config_reader.CheckError()){
        return -1;
    }
    // std::cout << config_reader << std::endl;
    /*** Set up the seaching object ***/
    MultiSamplingDubins multi_sampling_ltl;

    /*** Set the size of the workspace ***/
    double work_space_size_x = config_reader.GetReal("work_space_size_x", 0);
    double work_space_size_y = config_reader.GetReal("work_space_size_y", 0);
    multi_sampling_ltl.init_workspace(work_space_size_x, work_space_size_y);
    // Publish workspace size for visualization
    sampling::workspace_size_data space_data;
    space_data.size_x = work_space_size_x;
    space_data.size_y = work_space_size_y;
    lcm.publish("WORKSPACE", &space_data);

    /*** Set all parameters ***/
    // EPSILON is the forward step size when sampling searching
    double EPSILON = config_reader.GetReal("EPSILON", 0);
    // RADIUS is the radius of checking aera when sampling searching
    double RADIUS = config_reader.GetReal("RADIUS", 0);
    // min_radius is the minimum turning radius
    double min_radius = config_reader.GetReal("min_radius", 0);
    // time_step is the time step for way points sequence
    double time_step = config_reader.GetReal("time_step", 0);
    // Set the groud speed of the aircraft
    double ground_speed = 1;
    // collision_check_rate is the percentage that sampling nodes will do the collision checking
    double collision_check_rate = config_reader.GetReal("collision_check_rate", 0);

    multi_sampling_ltl.init_parameter(EPSILON, RADIUS, min_radius, ground_speed, time_step, collision_check_rate);

    /*** Read formula ***/
    // "(<> p0) && (<> p1) && (<> p2)" means visit p0, p1 and p2 region of interests
    // std::string ltl_formula = "(<> p0) && (<> p1) && (<> p2)";
    std::string ltl_formula = config_reader.Get("ltl_formula", "");
    // "<> (p0 && <> (p1 && (<> p2)))" means visit p0, p1 and p2 region of interests and by this order
    // std::string ltl_formula = "<> (p2 && <> (p1 && (<> p0)))";
    // Wrap all region of interests (ROI) as input for reading formula
    std::vector<std::string> buchi_regions;
    int num_buchi_regions = config_reader.GetInteger("num_buchi_regions", 0);

    for (int i = 0; i < num_buchi_regions; i++){
        buchi_regions.push_back(config_reader.Get("buchi_region_" + std::to_string(i), ""));
    }

    // indep_set store the ROI that independent to each other, in this case means p0, p1 and p2 have no intersections
    int num_indep_regions = config_reader.GetInteger("num_indep_regions", 0);

    // std::vector<int> indep_set = {0, 1, 2};
    std::vector<int> indep_set;
    for (int i = 0; i < num_indep_regions; i++){
        indep_set.push_back(config_reader.GetInteger("indep_region_" + std::to_string(i), -1));
    }

    multi_sampling_ltl.read_formula(ltl_formula, buchi_regions, indep_set);

    /*** Set the initial state of the UAV ***/
    int num_agents = config_reader.GetInteger("num_agents", 0);
    multi_sampling_ltl.set_num_agent(num_agents);
    std::vector<std::vector<double>> init_all_states;
    for (int i = 0; i < num_agents; i++){
        sampling::sample_data node_data;
        std::vector<double> init_state = {config_reader.GetReal("init_state_x_" + std::to_string(i), -1), config_reader.GetReal("init_state_y_" + std::to_string(i), -1), config_reader.GetReal("init_state_yaw_" + std::to_string(i), -1)};
        init_all_states.push_back(init_state);
        node_data.state[0] = init_state[0];
        node_data.state[1] = init_state[1];
        lcm.publish("SAMPLE", &node_data);
    }

    // sampling::sample_data node_data;
    // std::vector<double> init_state_1 = {20, 10, M_PI/2};
    // node_data.state[0] = init_state_1[0];
    // node_data.state[1] = init_state_1[1];
    // lcm.publish("SAMPLE", &node_data);
    // std::vector<double> init_state_2 = {80, 10, M_PI/2};
    // node_data.state[0] = init_state_2[0];
    // node_data.state[1] = init_state_2[1];
    // lcm.publish("SAMPLE", &node_data);
    // std::vector<std::vector<double>> init_all_states = {init_state_1, init_state_2};
    multi_sampling_ltl.set_init_state(init_all_states);
    
    /*** Set region of interests ***/
    // All ROI and obstacles are rectangle for now
    // Three parameters are x position, y position and the name of ROI (0 means p0)
    int num_ROI = config_reader.GetInteger("num_ROI", -1);
    sampling::region_data r_data;
    for (int i = 0; i < num_ROI; i++){
        std::string region_name = "ROI_" + std::to_string(i);
        std::pair <double, double> position_x (config_reader.GetReal("position_x_start_" + region_name, -1), config_reader.GetReal("position_x_end_" + region_name, -1));
        std::pair <double, double> position_y (config_reader.GetReal("position_y_start_" + region_name, -1), config_reader.GetReal("position_y_end_" + region_name, -1));
        multi_sampling_ltl.set_interest_region(position_x, position_y, config_reader.GetInteger("label_" + region_name, -1));
        // For visualization
        r_data.position_x[0] =  position_x.first;
        r_data.position_x[1] =  position_x.second;
        r_data.position_y[0] =  position_y.first;
        r_data.position_y[1] =  position_y.second;
        lcm.publish("REGION", &r_data);
    }

    /*** Set obstacles ***/
    int num_obstacles = config_reader.GetInteger("num_obstacles", -1);
    for (int i = 0; i < num_obstacles; i++){
        std::string obstacle_name = "obstacle_" + std::to_string(i);
        std::pair <double, double> position_x (config_reader.GetReal("position_x_start_" + obstacle_name, -1), config_reader.GetReal("position_x_end_" + obstacle_name, -1));
        std::pair <double, double> position_y (config_reader.GetReal("position_y_start_" + obstacle_name, -1), config_reader.GetReal("position_y_end_" + obstacle_name, -1));
        multi_sampling_ltl.set_obstacle(position_x, position_y);
        // For visualization
        r_data.position_x[0] =  position_x.first;
        r_data.position_x[1] =  position_x.second;
        r_data.position_y[0] =  position_y.first;
        r_data.position_y[1] =  position_y.second;
        lcm.publish("OBSTACLE", &r_data);
    }
    
    /*** Set the number of iterations ***/
    // Solution towards to optimal when iterations -> infinite
    int iterations = config_reader.GetInteger("iterations", -1);;
    /*** Start sampling searching ***/
    stopwatch.tic();
    // multi_sampling_ltl.start_sampling(iterations);
    multi_sampling_ltl.start_sampling();
    std::cout << "Time used for searching: " << stopwatch.toc() << std::endl;
    

    /*** Get result ***/
    // std::vector<std::vector<double>> path = ltl_sampling_dubins.get_path();
    std::vector<std::vector<WayPoint>> way_points = multi_sampling_ltl.get_waypoints();
    
    
    // Publish the trajectory and visualize the result
    for (int k = 0; k < num_agents; k++){
        sampling::path_data path_data_;
        path_data_.num_state = way_points[k].size();
        path_data_.state_x.resize(path_data_.num_state);
        path_data_.state_y.resize(path_data_.num_state);
        for (int i = 0; i < way_points[k].size(); i++) {
            path_data_.state_x[i] = way_points[k][i].x;
            path_data_.state_y[i] = way_points[k][i].y;
            // std::cout << "x: " << way_points[k][i].x << ", y: " << way_points[k][i].y << ", time: "<< way_points[k][i].t << std::endl;
        }
        lcm.publish("PATH", &path_data_);
    }
    sampling::sample_draw draw;
    draw.if_draw = true;
    lcm.publish("DRAW_SAMPLE", &draw);

    return 0;
}