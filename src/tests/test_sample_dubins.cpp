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
#include <fstream>
#include <string>

#include "trajectory/dubins_path.h"
#include "trans_sys/spot_hoa_interpreter.h"
#include "sampling/ltl_sampling_dubins.h"
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
    ConfigReader config_reader("../../config/test_sample_dubins.ini");
    // ConfigReader config_reader(filename);
    if (config_reader.CheckError()){
        return -1;
    }
    // std::cout << config_reader << std::endl;
    /*** Set up the seaching object ***/
    LTL_SamplingDubins ltl_sampling_dubins;

    /*** Set the size of the workspace ***/
    double work_space_size_x = config_reader.GetReal("work_space_size_x", 0);
    double work_space_size_y = config_reader.GetReal("work_space_size_y", 0);
    ltl_sampling_dubins.init_workspace(work_space_size_x, work_space_size_y);
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

    ltl_sampling_dubins.init_parameter(EPSILON, RADIUS, min_radius, ground_speed, time_step, collision_check_rate);

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

    ltl_sampling_dubins.read_formula(ltl_formula, buchi_regions, indep_set);

    /*** Set the initial state of the UAV ***/
    // std::vector<double> init_state = {50, 10, M_PI/2};
    std::vector<double> init_state;
    init_state.push_back(config_reader.GetReal("init_state_x", -1));
    init_state.push_back(config_reader.GetReal("init_state_y", -1));
    init_state.push_back(config_reader.GetReal("init_state_yaw", -1));
    ltl_sampling_dubins.set_init_state(init_state);
    
    /*** Set region of interests ***/
    // All ROI and obstacles are rectangle for now
    // Three parameters are x position, y position and the name of ROI (0 means p0)
    
    int num_ROI = config_reader.GetInteger("num_ROI", -1);
    sampling::region_data r_data;
    for (int i = 0; i < num_ROI; i++){
        std::string region_name = "ROI_" + std::to_string(i);
        std::pair <double, double> position_x (config_reader.GetReal("position_x_start_" + region_name, -1), config_reader.GetReal("position_x_end_" + region_name, -1));
        std::pair <double, double> position_y (config_reader.GetReal("position_y_start_" + region_name, -1), config_reader.GetReal("position_y_end_" + region_name, -1));
        ltl_sampling_dubins.set_interest_region(position_x, position_y, config_reader.GetInteger("label_" + region_name, -1));
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
        ltl_sampling_dubins.set_obstacle(position_x, position_y);
        // For visualization
        r_data.position_x[0] =  position_x.first;
        r_data.position_x[1] =  position_x.second;
        r_data.position_y[0] =  position_y.first;
        r_data.position_y[1] =  position_y.second;
        lcm.publish("OBSTACLE", &r_data);
    }
    
    /*** Set the number of iterations ***/
    // Solution towards to optimal when iterations -> infinite
    int iterations = config_reader.GetInteger("iterations", -1);
    /*** Start sampling searching ***/
    stopwatch.tic();
    std::cout << "Searching..." << std::endl;
    ltl_sampling_dubins.start_sampling(iterations);
    std::cout << "Time used for searching: " << stopwatch.toc() << std::endl;

    /*** Get result ***/
    // std::vector<std::vector<double>> path = ltl_sampling_dubins.get_path();
    std::vector<WayPoint> way_points = ltl_sampling_dubins.get_waypoints();

    // Publish the trajectory and visualize the result
    sampling::path_data path_data_;
    path_data_.num_state = way_points.size();
    path_data_.state_x.resize(path_data_.num_state);
    path_data_.state_y.resize(path_data_.num_state);
    for (int i = 0; i < way_points.size(); i++) {
        path_data_.state_x[i] = way_points[i].x;
        path_data_.state_y[i] = way_points[i].y;
        // std::cout << "x: " << way_points[i].x << ", y: " << way_points[i].y << ", time: "<< way_points[i].t << std::endl;
    }
    lcm.publish("PATH", &path_data_);
    sampling::sample_draw draw;
    draw.if_draw = true;
    lcm.publish("DRAW_SAMPLE", &draw);

    // Publish the way points data as planner output
    rts3a::routePlannerOutputs_t planner_output;
    planner_output.numWaypoints = way_points.size();
    planner_output.waypoints.resize(way_points.size());
    for (int i = 0; i < way_points.size(); i++){
        rts3a::waypoint_t way_point_;
        way_point_.x = way_points[i].x;
        way_point_.y = way_points[i].y;
        way_point_.z = 0;
        way_point_.t = way_points[i].t;
        std::cout << "x: " << way_points[i].x << ", y: " << way_points[i].y << ", t: " <<way_points[i].t << std::endl;
        
        planner_output.waypoints[i] = way_point_;
    }
    lcm.publish("PLANNER_OUTPUT", &planner_output);
    
    return 0;
}