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
#include "graph/graph.h"
#include "trans_sys/buchi_automaton.h"
#include "trans_sys/spot_hoa_interpreter.h"

// #include "sampling/sample_node.h"
#include "sampling/sample_space.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"

using namespace acel;

double fRand(double fMin, double fMax)
{
    std::uniform_real_distribution<double> dist(fMin, fMax);
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    return dist(rng);
}

std::vector<int> sample_from_ba(BAStruct buchi, SampleSpace &sample_space)
{
    if (sample_space.total_sample_num() == 0) {
        return std::vector<int>();
    }
    // srand(time(NULL));
    int size_buchi = buchi.state_num;
    std::vector<uint32_t> ba_accept_state = buchi.acc_state_idx;
    int new_ba_sample = rand() % (size_buchi);
    std::cout << "AAAAAA random!!! AAAAAAAAAAA:   " << new_ba_sample << std::endl;
    std::vector<int> new_ba_candidate;
    std::vector<int> temp_candidate;
    std::vector<std::vector<int>> act_q;
    // std::cout << "~~~~~~" << ba_accept_state[0] <<std::endl;
    if (new_ba_sample == ba_accept_state[0])
    {

        for (int i = 0; i < size_buchi; i++)
        {

            if (i != new_ba_sample && !buchi.trans_con[i][new_ba_sample].empty())
            {

                temp_candidate.push_back(i);
                if (sample_space.get_sub_space(i).num_samples() > 0)
                {
                    new_ba_candidate.push_back(i);
                    act_q.push_back({i, new_ba_sample});
                }
            }
        }
    }
    else
    {

        temp_candidate.push_back(new_ba_sample);
        // std::cout << "~~~~~~11" << std::endl;
        // std::cout << "num::::" << sample_space.get_sub_space(new_ba_sample).num_samples() << std::endl;
        if (sample_space.get_sub_space(new_ba_sample).num_samples() > 0)
        {
            // std::cout << "~~~~~~" << std::endl;
            new_ba_candidate.push_back(new_ba_sample);
            act_q.push_back({new_ba_sample, new_ba_sample});
        }
    }
    while (new_ba_candidate.empty())
    {
        std::vector<int> new_temp_candidate;
        for (int i = 0; i < temp_candidate.size(); i++)
        {
            for (int j = 0; j < size_buchi; j++)
            {
                if (j != new_ba_sample && !buchi.trans_con[j][temp_candidate[i]].empty() && std::find(temp_candidate.begin(), temp_candidate.end(), j) == temp_candidate.end())
                {
                    new_temp_candidate.push_back(j);
                    if (sample_space.get_sub_space(j).num_samples() > 0)
                    {
                        // std::cout << "~!!!" << j << std::endl;
                        new_ba_candidate.push_back(j);
                        act_q.push_back({j, temp_candidate[i]});
                    }
                }
            }
        }
        temp_candidate = new_temp_candidate;
        // std::cout << "~~~back" << std::endl;
    }
    // for (auto &i:new_ba_candidate) {
    //     std::cout << i << " ";
    // }
    // srand(time(NULL));
    int r = rand() % (new_ba_candidate.size());
    // std::cout << "random num: " << r << std::endl;
    int new_ba_state = new_ba_candidate[r];
    // std::cout << "new_ba_state: " << new_ba_state << std::endl;
    std::cout << "BBBBBB sampled buchi state:    " << act_q[r][0] << ", and: "<< act_q[r][1] << std::endl;
    return act_q[r];
}


void buchi_post (BAStruct &ba, std::vector<int> indep_set) {
    std::bitset<32> indep_bitset;
    for (auto it = indep_set.begin(); it != indep_set.end(); it++) {
        indep_bitset.set(*it, 1);
    }
    // std::cout << "indep_bitset: " << indep_bitset << std::endl;
    int alphabet_set_size = ba.alphabet_set.size();
    std::vector<int> indep_alphabet;
    for (int i = 0; i < alphabet_set_size; i++) {
        std::bitset<32> check_indep = std::bitset<32>(ba.alphabet_set[i]) & indep_bitset;
        if (check_indep.count() > 1) {
            indep_alphabet.push_back(i);
        }
    }

    // std::cout << "indep_alphabet: " << std::endl;
    // for (auto it = indep_alphabet.begin(); it != indep_alphabet.end(); it++) {
    //     std::cout << *it << " ";
    // }
    // std::cout << std::endl;

    

    for (int i = 0; i < ba.state_num; i++)
    {
        for (auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
        {
            std::vector<int> inter;
            std::set_intersection((*it).begin(), (*it).end(), indep_alphabet.begin(), indep_alphabet.end(), std::back_inserter(inter));
            (*it).erase(std::set_difference((*it).begin(), (*it).end(), inter.begin(), inter.end(), (*it).begin()), (*it).end());

            // std::string str = "";
            // for (auto itc = (*it).begin(); itc != (*it).end(); itc++)
            // {
            //     str = str + std::to_string(*itc) + ",";
            // }
            // std::cout << std::setw(25) << str << " ";
        }
        // std::cout << "   -----   line " << i << std::endl;
    }

}

bool if_in_region (std::vector<double> state, Region region) {
    if (state[0] > region.get_x_position().first && state[0] < region.get_x_position().second && state[1] > region.get_y_position().first && state[1] < region.get_y_position().second) {
        return true;
    }
    else {
        return false;
    }
}

double get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[1] - states_2[1], 2));
    return dist;
}


std::vector<double> step_from_to (SampleNode parent_sample, std::vector<double> sampled_state, double EPSILON) {
    if (get_dist(parent_sample.get_state(), sampled_state) < EPSILON) {
        return sampled_state;
    }
    else {
        double theta = atan2(sampled_state[1] - parent_sample.get_state()[1], sampled_state[0] - parent_sample.get_state()[0]);
        std::vector<double> new_state = {parent_sample.get_state()[0] + EPSILON*cos(theta), parent_sample.get_state()[1] + EPSILON*sin(theta)};
        return new_state;
    }
}

int step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions) {
    std::bitset<32> bit_set;
    std::cout << "*******in step from to buchi function: ******" << std::endl;
    std::cout << "parent ba: " << paraent_ba << std::endl;
    int buchi_num = paraent_ba;
    for (int i = 0; i < all_interest_regions.size(); i++) {
        if (if_in_region(new_sample_state, all_interest_regions.find(i)->second)) {
            // std::cout << "(step_from_to_buchi function) interested region num: " << all_interest_regions.find(i)->second.get_region_interest() << std::endl;
            bit_set.set(all_interest_regions.find(i)->second.get_region_interest());
            std::cout << "new state is in region: " << all_interest_regions.find(i)->second.get_region_interest() << std::endl;
            break;
        }
    }
    int act = bit_set.to_ullong();
    // std::cout << "(step_from_to_buchi function) paraent buchi state: " << paraent_ba << std::endl;
    for (int i = 0; i < ba.trans_con[paraent_ba].size(); i++) {
        if (std::find(ba.trans_con[paraent_ba][i].begin(), ba.trans_con[paraent_ba][i].end(), act) != ba.trans_con[paraent_ba][i].end()) {
            buchi_num = i;
            break;
        }
    }
    std::cout << "new ba: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) out buchi state: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) test bit set: " << bit_set << std::endl;
    // std::cout << "(step_from_to_buchi function) test int: " << bit_set.to_ullong() << std::endl;
    std::cout << "****************************************" << std::endl;
    return buchi_num;
}



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
    SpotHoaInterpreter ltl2ba_lib;
    BAStruct ba = ltl2ba_lib.GetBuchi(ltl_formula, buchi_regions);

    std::vector<int> indep_set = {0, 1, 2};


    // std::cout << "Init buchi state: " << ba.init_state_idx << std::endl;
	// std::cout << " - Transition Table String: " << std::endl;
	// for(auto it = ba.trans_table_str.begin(); it != ba.trans_table_str.end(); it++)
	// {
	// 	for(auto ite = (*it).begin(); ite != (*it).end(); ite++)
	// 		std::cout << std::setw(25) << (*ite) << " ";
	// 	std::cout << std::endl;
	// }
    // std::cout << "\nAlphabet Set: " << std::endl;
    // int idx = 0;
    // for (auto it = ba.alphabet_set.begin(); it != ba.alphabet_set.end(); it++)
    // {
    //     std::cout << *it << ", " << std::bitset<32>(*it) << " , " << idx << std::endl;
    //     idx++;
    // }
    // std::cout << "\nTransition Table: " << std::endl;
    // for (int i = 0; i < ba.state_num; i++)
    // {
    //     for (auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
    //     {
    //         std::string str = "";
    //         for (auto itc = (*it).begin(); itc != (*it).end(); itc++)
    //         {
    //             str = str + std::to_string(*itc) + ",";
    //         }
    //         std::cout << std::setw(25) << str << " ";
    //     }
    //     std::cout << "   -----   line " << i << std::endl;
    // }

    // std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula, buchi_regions);
    buchi_post(ba, indep_set);
    SampleSpace all_space(ba.state_num);
    int init_ba = ba.init_state_idx;
    int acc_ba = ba.acc_state_idx.front();
    std::cout << "init buchi state: " << init_ba << std::endl;
    std::cout << "accept buchi state: " << acc_ba << std::endl;
    // std::vector<double> init_state = {1.4, 3.2};
    // std::vector<double> init_state = {50, 10, M_PI};
    std::vector<double> init_state = {50, 10};
    // init_state.push_back(50);
    // init_state.push_back(10);
    // init_state.push_back(M_PI);

    SampleNode init_node;
    init_node.set_id(0);
    init_node.set_state(init_state);
    init_node.set_ba(ba.init_state_idx);
    init_node.set_cost(0.0);
    std::cout << "initial node id: " << init_node.get_id() << ", node 1 state: " << init_node.get_state()[0] << std::endl;
    all_space.insert_sample(init_node, init_ba);
    std::cout << "all sample " << init_ba << " space size: " << all_space.get_sub_space(init_ba).num_samples() << std::endl;
    // SubSampleSpace sub_space;
    std::cout << "~~~~~~~~~" << std::endl;
    // all_space.insert_sample(node_1, 7);
    // sub_space.insert_sample(node_1);
    // std::cout << "sub sample space 1 size: " << sub_space.num_samples() << std::endl;
    // std::cout << "all sample space 7 size: " << all_space.get_sub_space(7).num_samples() << std::endl;
    // std::cout << "all sample space 6 size: " << all_space.get_sub_space(6).num_samples() << std::endl;


    

    int idx = 0;
    for (auto it = ba.alphabet_set.begin(); it != ba.alphabet_set.end(); it++)
    {
        std::cout << *it << ", " << std::bitset<32>(*it) << " , " << idx << std::endl;
        idx++;
    }
    

    
    std::cout << "\nTransition Table: " << std::endl;
    for (int i = 0; i < ba.state_num; i++)
    {
        for (auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
        {
            std::string str = "";
            for (auto itc = (*it).begin(); itc != (*it).end(); itc++)
            {
                str = str + std::to_string(*itc) + ",";
            }
            std::cout << std::setw(8) << str << " ";
        }
        std::cout << "   -----   line " << i << std::endl;
    }

    // SubSampleSpace space_1;
    // space_1.insert_sample(node_1);
    // SampleNode node_2 = space_1.get_sample(2);
    // node_2.set_id(3);
    // space_1.insert_sample(node_2);
    // std::cout << "from node: " << std::endl;
    // std::cout << "node 2 id: " << node_2.get_id() << ", node 2 state: " << node_2.get_states()[0] << std::endl;
    // std::cout << "from space: " << std::endl;
    // std::cout << "node 2 id: " << space_1.get_sample(2).get_id() << ", node 2 state: " << space_1.get_sample(3).get_states()[1] << std::endl;
    // std::bitset<32> f1;
    // std::bitset<32> f2;
    // std::bitset<32> f3 = f1&f2;
    // f1.set(0,1);
    // f2.set(1,1);
    // std::cout << "f1111: " << f1 << std::endl;
    // std::cout << "f2222: " << f2 << std::endl;
    // std::cout << "f3333: " << f3 << std::endl;
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    sampling::region_data r_data;
    sampling::all_regions all_regions_data;

    Region interest_0;
    Region interest_1;
    Region interest_2;
    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    interest_0.set_position(position_x, position_y);
    interest_0.set_region_interest(0);
    
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    interest_1.set_position(position_x, position_y);
    interest_1.set_region_interest(1);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);


    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    interest_2.set_position(position_x, position_y);
    interest_2.set_region_interest(2);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    lcm.publish("REGION", &r_data);

    std::cout << "interest 0 position x: " << interest_0.get_x_position().first << ", " << interest_0.get_x_position().second << std::endl;
    std::cout << "interest 0 position y: " << interest_0.get_y_position().first << ", " << interest_0.get_y_position().second << std::endl;
    std::cout << "interest 0 ba interest: " << interest_0.get_region_interest() << std::endl;

    std::cout << "interest 1 position x: " << interest_1.get_x_position().first << ", " << interest_1.get_x_position().second << std::endl;
    std::cout << "interest 1 position y: " << interest_1.get_y_position().first << ", " << interest_1.get_y_position().second << std::endl;
    std::cout << "interest 1 ba interest: " << interest_1.get_region_interest() << std::endl;

    std::cout << "interest 2 position x: " << interest_2.get_x_position().first << ", " << interest_2.get_x_position().second << std::endl;
    std::cout << "interest 2 position y: " << interest_2.get_y_position().first << ", " << interest_2.get_y_position().second << std::endl;
    std::cout << "interest 2 ba interest: " << interest_2.get_region_interest() << std::endl;

    // std::vector<Region> all_interest_regions;
    std::map<int, Region> all_interest_regions;
    all_interest_regions[0] = interest_0;
    all_interest_regions[1] = interest_1;
    all_interest_regions[2] = interest_2;

    // all_interest_regions.push_back(interest_0);
    // all_interest_regions.push_back(interest_1);
    // all_interest_regions.push_back(interest_2);
    bool find_path = false;
    uint64_t first_acc_state_id;
    int iteration = 300;
    
    for (int i = 0; i < iteration; i++) {
        std::cout << "=============New Iteration============================" << std::endl;
        std::vector<int> ba_act = sample_from_ba(ba, all_space);
        double new_node_x = 0;
        double new_node_y = 0;
        if (ba_act[0] == ba_act[1]) {
            new_node_x = fRand(0, work_space_size_x);
            new_node_y = fRand(0, work_space_size_y);
        }
        else {
            int interest_id = 0;
            int act = ba.trans_con[ba_act[0]][ba_act[1]].front();
            // std::cout << "act region: " << act << std::endl;
            std::bitset<32> act_bit = std::bitset<32>(act);
            // std::cout << "act region bit: " << act_bit << std::endl;
            for (int i = 0; i < act_bit.size(); i++) {
                if (act_bit.test(i)){
                    interest_id = i;
                    break;
                }
            }
            double new_node_x_min = all_interest_regions.find(interest_id)->second.get_x_position().first;
            double new_node_x_max = all_interest_regions.find(interest_id)->second.get_x_position().second;
            double new_node_y_min = all_interest_regions.find(interest_id)->second.get_y_position().first;
            double new_node_y_max = all_interest_regions.find(interest_id)->second.get_y_position().second;
            new_node_x = fRand(new_node_x_min, new_node_x_max);
            new_node_y = fRand(new_node_y_min, new_node_y_max);
            // std::cout << "=============================================" << std::endl;
            std::cout << "x min: " << new_node_x_min << "x max: " << new_node_x_max << std::endl;
            std::cout << "y min: " << new_node_y_min << "x max: " << new_node_y_max << std::endl;
            
            std::cout << "randomed interest region id: " << interest_id << std::endl;
        }
        std::cout  << "!!!!!Sampled ba state: " << ba_act[0] << std::endl;
        std::cout << "random x: " << new_node_x << std::endl;
        std::cout << "random y: " << new_node_y << std::endl;
        std::vector<double> sampled_position = {new_node_x, new_node_y}; 
        SampleNode parent_sample = all_space.get_sub_space(ba_act[0]).get_parent(sampled_position);
        // std::cout << "===== size: " << parent_sample.get_state().size() << std::endl;
        std::cout << "paraent node x: " << parent_sample.get_state()[0] << ", paraent node y: " << parent_sample.get_state()[1] << std::endl;
        std::vector<double> new_sample_state = step_from_to(parent_sample, sampled_position, EPSILON);
        std::cout << "new sample state x: " << new_sample_state[0] << ", new sample state y: " << new_sample_state[1] << std::endl;
        
        // std::vector<double> test_position = {15.0, 85.0};
        int new_ba = step_from_to_buchi(parent_sample.get_ba(), new_sample_state, ba, all_interest_regions);
        /// Choose parent

        // std::vector<int> parent_ba_candidate;
        // if (new_ba == parent_sample.get_ba()) {
        //     parent_ba_candidate.push_back(new_ba);
        // }
        // else {
        //     parent_ba_candidate.push_back(parent_sample.get_ba());
        //     parent_ba_candidate.push_back(new_ba);
        // }
        // SampleNode chosen_parent_sample = parent_sample;
        // std::cout << "!!!!!dist between two node: " << get_dist(chosen_parent_sample.get_state(), new_sample_state) << std::endl;
        
        SampleNode chosen_parent_sample = all_space.get_sub_space(parent_sample.get_ba()).rechoose_parent(parent_sample, new_sample_state, RADIUS);



        SampleNode new_node;
        uint64_t new_id = all_space.get_sub_space(new_ba).num_samples();
        new_node.set_id(new_id);
        new_node.set_ba(new_ba);
        new_node.set_state(new_sample_state);
        new_node.set_cost(chosen_parent_sample.get_cost() + get_dist(chosen_parent_sample.get_state(), new_sample_state));
        // std::cout << "parent ba state: " << new_ba << std::endl;
        new_node.set_parent_ba(chosen_parent_sample.get_ba());
        new_node.set_parent_id(chosen_parent_sample.get_id());
        all_space.insert_sample(new_node, new_ba);
        
        sampling::sample_data node_data;
        node_data.state[0] = new_sample_state[0];
        node_data.state[1] = new_sample_state[1];
        lcm.publish("SAMPLE", &node_data);

        std::cout << "new ba state: " << new_ba << std::endl;
        if (new_ba == ba.acc_state_idx.front()) {
            // std::cout << "acc ba: " << ba.acc_state_idx.front() << std::endl;
            std::cout << "Find a solution!!!" << std::endl;
            first_acc_state_id = new_id;
            find_path = true;
            break;
        }


        std::cout << "total sample num: " << all_space.total_sample_num() << std::endl;
        // std::cout << "sample buchi 1: " << ba_act[0] << ", sample buchi 2: " << ba_act[1] << std::endl;
        // std::cout << "double random: " << fRand(0, 100) << std::endl;

    }
    
    
    /// path generate
    std::vector<std::vector<double>> path;
    int current_ba = acc_ba;
    uint64_t current_id = first_acc_state_id;
    std::cout << "last ba id: " << acc_ba << std::endl;
    std::cout << "last id " << current_id << std::endl;

    // SampleNode test_node = all_space.get_sub_space(0).get_sample(0);
    // std::cout << "WWWWWWWWWWWWWWWWWWWWWWWFFFFFFFF************************" << std::endl;
    if (find_path) {
        
        while (current_ba != init_ba || current_id != 0) {
            
            SampleNode current_node = all_space.get_sub_space(current_ba).get_sample(current_id);
            
            std::vector<double> path_node = current_node.get_state();
            path.push_back(path_node);
            
            current_ba = current_node.get_parent_ba();
            current_id = current_node.get_parent_id();
            // std::cout << "current ba id: " << current_ba << std::endl;
            // std::cout << "current id " << current_id << std::endl;
            
        }
    }
    // std::cout << "WWWWWWWWWWWWWWWWWWWWWWWFFFFFFFF************************"  << std::endl;
    std::reverse(path.begin(), path.end());
    

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