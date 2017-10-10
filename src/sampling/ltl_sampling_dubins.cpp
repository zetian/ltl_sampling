#include <random>
#include <algorithm>
#include <bitset>
#include <climits>

#include "sampling/ltl_sampling_dubins.h"

double LTL_SamplingDubins::get_dist_dubins(std::vector<double> states_1, std::vector<double> states_2, double RADIUS_L_, double RADIUS_R_) {
    double min_length = DubinsSteer::GetDubinsCurveLength(states_1, states_2, RADIUS_L_, RADIUS_R_);
    return min_length;
}

double LTL_SamplingDubins::fRand(double fMin, double fMax)
{
    std::uniform_real_distribution<double> dist(fMin, fMax);
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    return dist(rng);
}

std::vector<int> LTL_SamplingDubins::sample_from_ba(BAStruct buchi, SampleSpace &sample_space)
{
    if (sample_space.total_sample_num() == 0) {
        return std::vector<int>();
    }
    int size_buchi = buchi.state_num;
    int temp = buchi.acc_state_idx.front();
    if (size_buchi == 1) {
        return {temp, temp};
    }
    std::vector<uint32_t> ba_accept_state = buchi.acc_state_idx;
    int new_ba_sample = rand() % (size_buchi);
    std::vector<int> new_ba_candidate;
    std::vector<int> temp_candidate;
    std::vector<std::vector<int>> act_q;
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
        if (sample_space.get_sub_space(new_ba_sample).num_samples() > 0)
        {
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
                        new_ba_candidate.push_back(j);
                        act_q.push_back({j, temp_candidate[i]});
                    }
                }
            }
        }
        temp_candidate = new_temp_candidate;
    }
    int r = rand() % (new_ba_candidate.size());
    int new_ba_state = new_ba_candidate[r];
    return act_q[r];
}


void LTL_SamplingDubins::buchi_post (BAStruct &ba, std::vector<int> indep_set) {
    if (indep_set.size() == 1){
        return;
    }
    std::bitset<32> indep_bitset;
    for (auto it = indep_set.begin(); it != indep_set.end(); it++) {
        indep_bitset.set(*it, 1);
    }
    int alphabet_set_size = ba.alphabet_set.size();
    std::vector<int> indep_alphabet;
    for (int i = 0; i < alphabet_set_size; i++) {
        std::bitset<32> check_indep = std::bitset<32>(ba.alphabet_set[i]) & indep_bitset;
        if (check_indep.count() > 1) {
            indep_alphabet.push_back(i);
        }
    }

    for (int i = 0; i < ba.state_num; i++)
    {
        for (auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
        {
            std::vector<int> inter;
            std::set_intersection((*it).begin(), (*it).end(), indep_alphabet.begin(), indep_alphabet.end(), std::back_inserter(inter));
            (*it).erase(std::set_difference((*it).begin(), (*it).end(), inter.begin(), inter.end(), (*it).begin()), (*it).end());
        }
    }

}

bool LTL_SamplingDubins::if_in_region (std::vector<double> state, Region region) {
    if (state[0] > region.get_x_position().first && state[0] < region.get_x_position().second && state[1] > region.get_y_position().first && state[1] < region.get_y_position().second) {
        return true;
    }
    else {
        return false;
    }
}

std::vector<double> LTL_SamplingDubins::step_from_to (SampleNode parent_sample, std::vector<double> sampled_state, DubinsSteer::SteerData& dubins_steer_data, double EPSILON_) {
    dubins_steer_data = DubinsSteer::GetDubinsTrajectoryPointWise(parent_sample.get_state(), sampled_state, radius_L_, radius_R_);
    // if (dubins_steer_data.traj_point_wise.size() < 30) {
    //     std::cout << "something wrong" << std::endl;
    //     return std::vector<double>();
    // }
    if (dubins_steer_data.traj_length < EPSILON_) {
        return sampled_state;
    }
    else {
        int id = 0;
        for (int i = 0; i < dubins_steer_data.traj_len_map.size(); i++) {
            if (dubins_steer_data.traj_len_map[i] > EPSILON_) {
                id = i;
                break;
            }
        }
        std::vector<double> new_state = dubins_steer_data.traj_point_wise[id];
        std::vector<std::vector<double>> new_traj(dubins_steer_data.traj_point_wise.begin(), dubins_steer_data.traj_point_wise.begin() + id);
        // traj_point_wise = new_traj;
        dubins_steer_data.traj_point_wise = new_traj;
        dubins_steer_data.traj_length = dubins_steer_data.traj_len_map[id];
         return new_state;
    }
}

int LTL_SamplingDubins::step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions) {
    std::bitset<32> bit_set;
    // std::cout << "*******in step from to buchi function: ******" << std::endl;
    int buchi_num = paraent_ba;
    for (int i = 0; i < all_interest_regions.size(); i++) {
        if (if_in_region(new_sample_state, all_interest_regions.find(i)->second)) {
            bit_set.set(all_interest_regions.find(i)->second.get_region_interest());
            break;
        }
    }
    int act = bit_set.to_ullong();
    for (int i = 0; i < ba.trans_con[paraent_ba].size(); i++) {
        if (std::find(ba.trans_con[paraent_ba][i].begin(), ba.trans_con[paraent_ba][i].end(), act) != ba.trans_con[paraent_ba][i].end()) {
            buchi_num = i;
            break;
        }
    }
    // std::cout << "new ba: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) out buchi state: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) test bit set: " << bit_set << std::endl;
    // std::cout << "(step_from_to_buchi function) test int: " << bit_set.to_ullong() << std::endl;
    // std::cout << "****************************************" << std::endl;
    return buchi_num;
}

void LTL_SamplingDubins::read_formula(std::string ltl_formula, std::vector<std::string> buchi_regions, std::vector<int> indep_set) {
    SpotHoaInterpreter ltl2ba_lib;
    ba_ = ltl2ba_lib.GetBuchi(ltl_formula, buchi_regions);
    buchi_post(ba_, indep_set);
}
void LTL_SamplingDubins::init_workspace(double work_space_size_x, double work_space_size_y) {
    work_space_size_x_ = work_space_size_x;
    work_space_size_y_ = work_space_size_y;
}

void LTL_SamplingDubins::init_parameter(double EPSILON, double RADIUS, double radius_L, double radius_R){
    EPSILON_ = EPSILON;
    RADIUS_ = RADIUS;
    radius_L_ = radius_L;
    radius_R_ = radius_R;
}

std::vector<double> LTL_SamplingDubins::sample_state(std::vector<int> ba_act) {
    double new_node_x = 0;
    double new_node_y = 0;
    if (ba_act[0] == ba_act[1]) {
        new_node_x = fRand(0, work_space_size_x_);
        new_node_y = fRand(0, work_space_size_y_);
    }
    else {
        int interest_id = 0;
        int act = ba_.trans_con[ba_act[0]][ba_act[1]].front();
        std::bitset<32> act_bit = std::bitset<32>(act);
        for (int i = 0; i < act_bit.size(); i++) {
            if (act_bit.test(i)){
                interest_id = i;
                break;
            }
        }
        double new_node_x_min = all_interest_regions_.find(interest_id)->second.get_x_position().first;
        double new_node_x_max = all_interest_regions_.find(interest_id)->second.get_x_position().second;
        double new_node_y_min = all_interest_regions_.find(interest_id)->second.get_y_position().first;
        double new_node_y_max = all_interest_regions_.find(interest_id)->second.get_y_position().second;
        new_node_x = fRand(new_node_x_min, new_node_x_max);
        new_node_y = fRand(new_node_y_min, new_node_y_max);
    }
    double yaw = fRand(0, M_PI*2);
    std::vector<double> sampled_position = {new_node_x, new_node_y, yaw}; 
    return sampled_position;
}

void LTL_SamplingDubins::set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, int interest_id) {
    Region interest_region;
    interest_region.set_position(position_x, position_y);
    interest_region.set_region_interest(interest_id);
    all_interest_regions_[interest_id] = interest_region;
}


void LTL_SamplingDubins::set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y){
    Region obstacle;
    obstacle.set_position(position_x, position_y);
    all_obstacles_.push_back(obstacle);
}

void LTL_SamplingDubins::set_init_state(std::vector<double> init_state) {
    all_space_.set_space(ba_.state_num);
    int init_ba = ba_.init_state_idx;
    int acc_ba = ba_.acc_state_idx.front();
    SampleNode init_node;
    init_node.set_id(0);
    init_node.set_state(init_state);
    init_node.set_ba(ba_.init_state_idx);
    init_node.set_cost(0.0);
    all_space_.insert_sample(init_node, init_ba);
}

void LTL_SamplingDubins::start_sampling(int iteration) {
    bool find_path = false;
    uint64_t first_acc_state_id;
    lcm::LCM lcm;
    for (int i = 0; i < iteration; i++) {
        // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        // std::cout << "iteration: " << i << std::endl;
        std::vector<int> ba_act = sample_from_ba(ba_, all_space_);
        std::vector<double> sampled_position = sample_state(ba_act);
        DubinsSteer::SteerData dubins_steer_data;
        SampleNode parent_sample = all_space_.get_sub_space(ba_act[0]).get_parent_dubins(sampled_position, radius_L_, radius_R_);
        std::vector<double> new_sample_state = step_from_to(parent_sample, sampled_position, dubins_steer_data, EPSILON_);
        if (Region::collision_check_dubins(dubins_steer_data.traj_point_wise, all_obstacles_, work_space_size_x_, work_space_size_y_)) {
            continue;
        } 
        int new_ba = step_from_to_buchi(parent_sample.get_ba(), new_sample_state, ba_, all_interest_regions_);
        SampleNode &chosen_parent_sample = all_space_.get_sub_space(parent_sample.get_ba()).rechoose_parent_dubins(parent_sample, new_sample_state, dubins_steer_data, all_obstacles_, work_space_size_x_, work_space_size_y_, RADIUS_, radius_L_, radius_R_);
        SampleNode new_node;
        uint64_t new_id = all_space_.get_sub_space(new_ba).num_samples();
        new_node.set_id(new_id);
        new_node.set_ba(new_ba);
        chosen_parent_sample.add_children_id(std::make_pair(new_ba, new_id));
        new_node.set_state(new_sample_state);
        new_node.set_cost(chosen_parent_sample.get_cost() + dubins_steer_data.traj_length);
        new_node.set_parent_ba(chosen_parent_sample.get_ba());
        new_node.set_parent_id(chosen_parent_sample.get_id());
        new_node.set_traj(dubins_steer_data.traj_point_wise);

        all_space_.insert_sample(new_node, new_ba);

        all_space_.rewire_dubins(new_id, new_ba, all_obstacles_, work_space_size_x_, work_space_size_y_, RADIUS_, radius_L_, radius_R_);

        /// Vis for debug
        // sampling::sample_data node_data;
        // node_data.state[0] = new_sample_state[0];
        // node_data.state[1] = new_sample_state[1];
        // lcm.publish("SAMPLE", &node_data);

        // std::cout << "new ba state: " << new_ba << std::endl;
        // if (new_ba == ba_.acc_state_idx.front()) {
        //     // std::cout << "acc ba: " << ba.acc_state_idx.front() << std::endl;
        //     std::cout << "Find a solution!!!" << std::endl;
        //     first_acc_state_id = new_id;
        //     find_path = true;
        //     break;
        // }
    }
}

std::vector<std::vector<double>> LTL_SamplingDubins::get_path() {
    lcm::LCM lcm;
    bool find_path = false;
    int current_ba = ba_.acc_state_idx.front();
    int init_ba = ba_.init_state_idx;

    std::vector<std::vector<double>> path_nodes_sq;

    if (all_space_.get_sub_space(current_ba).num_samples() > 0) {
        find_path = true;
    }
    else {
        std::cout << "No feasible path found. " << std::endl;
        return path_;
    }
    
    double total_cost = INT_MAX;
    uint64_t current_id = 1;
    if (find_path) {
        SampleNode min_cost_sample = all_space_.get_sub_space(ba_.acc_state_idx.front()).get_min_cost_sample();
        // sampling::sample_data node_data;
        // node_data.state[0] = min_cost_sample.get_state()[0];
        // node_data.state[1] = min_cost_sample.get_state()[1];
        // lcm.publish("SAMPLE", &node_data);

        std::cout << "Path cost is: " << min_cost_sample.get_cost() << std::endl;
        current_id = min_cost_sample.get_id();
        while (current_ba != init_ba || current_id != 0) {
            SampleNode current_node = all_space_.get_sub_space(current_ba).get_sample(current_id);
            // sampling::sample_data node_data;
            // node_data.state[0] = current_node.get_state()[0];
            // node_data.state[1] = current_node.get_state()[1];
            // lcm.publish("SAMPLE", &node_data);
            std::vector<std::vector<double>> temp = current_node.get_traj();
            std::reverse(temp.begin(), temp.end());
            path_nodes_sq.insert( path_nodes_sq.end(), temp.begin(), temp.end() );
            current_ba = current_node.get_parent_ba();
            current_id = current_node.get_parent_id();
            
        }
        std::reverse(path_nodes_sq.begin(), path_nodes_sq.end());
    
    }
    path_ = path_nodes_sq;
    return path_nodes_sq;
}

double LTL_SamplingDubins::get_path_length(){
    if (all_space_.get_sub_space(ba_.acc_state_idx.front()).num_samples() > 0) {
        SampleNode min_cost_sample = all_space_.get_sub_space(ba_.acc_state_idx.front()).get_min_cost_sample();
        return min_cost_sample.get_cost();
    }
    else {
        return INT_MAX;
    }
}