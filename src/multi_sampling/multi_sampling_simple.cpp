#include <random>
#include <algorithm>
#include <bitset>
#include <climits>
#include "multi_sampling/multi_sampling_simple.h"

double MultiSamplingSimple::get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[1] - states_2[1], 2));
    return dist;
}

double MultiSamplingSimple::get_dist(MultiSampleNode multi_sample_1, MultiSampleNode multi_sample_2){
    std::vector<std::vector<double>> states_1 = multi_sample_1.get_all_states();
    std::vector<std::vector<double>> states_2 = multi_sample_2.get_all_states();
    if (states_1.size() != states_2.size()){
        return -1;
    }
    double dist = 0;
    for (int i = 0; i < states_1.size(); i++) {
        dist = dist + sqrt(pow(states_1[i][0] - states_2[i][0], 2) + pow(states_1[i][1] - states_2[i][1], 2));
    }
    return dist;
}

double MultiSamplingSimple::get_dist(std::vector<std::vector<double>> states_1, std::vector<std::vector<double>> states_2){
    if (states_1.size() != states_2.size()){
        return -1;
    }
    double dist = 0;
    for (int i = 0; i < states_1.size(); i++) {
        dist = dist + sqrt(pow(states_1[i][0] - states_2[i][0], 2) + pow(states_1[i][1] - states_2[i][1], 2));
    }
    return dist;
}

double MultiSamplingSimple::fRand(double fMin, double fMax)
{
    std::uniform_real_distribution<double> dist(fMin, fMax);
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    return dist(rng);
}





std::vector<int> MultiSamplingSimple::sample_from_ba(BAStruct buchi, SampleSpace &sample_space)
{
    if (sample_space.total_sample_num() == 0) {
        return std::vector<int>();
    }
    int size_buchi = buchi.state_num;
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

void MultiSamplingSimple::set_num_agent(int num_agent){
    num_agent_ = num_agent;
}

void MultiSamplingSimple::buchi_post (BAStruct &ba, std::vector<int> indep_set) {
    if (indep_set.size() == 1 || indep_set.size() == 0){
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

bool MultiSamplingSimple::if_in_region (std::vector<double> state, Region region) {
    if (state[0] > region.get_x_position().first && state[0] < region.get_x_position().second && state[1] > region.get_y_position().first && state[1] < region.get_y_position().second) {
        return true;
    }
    else {
        return false;
    }
}

std::vector<double> MultiSamplingSimple::step_from_to (MultiSampleNode parent_sample, std::vector<double> sampled_state) {
    if (get_dist(parent_sample.get_state(), sampled_state) < EPSILON_) {
        return sampled_state;
    }
    else {
        double theta = atan2(sampled_state[1] - parent_sample.get_state()[1], sampled_state[0] - parent_sample.get_state()[0]);
        std::vector<double> new_state = {parent_sample.get_state()[0] + EPSILON_*cos(theta), parent_sample.get_state()[1] + EPSILON_*sin(theta)};
        return new_state;
    }
}


// std::vector<std::vector<double>> MultiSamplingSimple::step_from_to (MultiSampleNode parent_sample, std::vector<std::vector<double>> all_sampled_states){
//     // std::vector<std::vector<double>> all_new_states;
//     if (get_dist(parent_sample.get_all_states(), all_sampled_states) < EPSILON_) {
//         return all_sampled_states;
//     }
//     else {
//         std::vector<std::vector<double>> all_new_states;
//         for (int i = 0; i < all_sampled_states.size(); i++){
//             double theta = atan2(all_sampled_states[i][1] - parent_sample.get_all_states()[i][1], all_sampled_states[i][0] - parent_sample.get_all_states()[i][0]);
//             std::vector<double> new_state = {parent_sample.get_all_states()[i][0] + EPSILON_*cos(theta), parent_sample.get_all_states()[i][1] + EPSILON_*sin(theta)};
//             all_new_states.push_back(new_state);
//         }
//         return all_new_states;
//     }
// }

std::vector<std::vector<double>> MultiSamplingSimple::step_from_to (MultiSampleNode parent_sample, std::vector<std::vector<double>> all_sampled_states){

    std::vector<std::vector<double>> all_new_states;
    for (int i = 0; i < all_sampled_states.size(); i++){
        if (get_dist(parent_sample.get_all_states()[i], all_sampled_states[i]) < EPSILON_){
            all_new_states.push_back(all_sampled_states[i]);
            continue;
        }
        double theta = atan2(all_sampled_states[i][1] - parent_sample.get_all_states()[i][1], all_sampled_states[i][0] - parent_sample.get_all_states()[i][0]);
        std::vector<double> new_state = {parent_sample.get_all_states()[i][0] + EPSILON_*cos(theta), parent_sample.get_all_states()[i][1] + EPSILON_*sin(theta)};
        all_new_states.push_back(new_state);
    }
    return all_new_states;

}


int MultiSamplingSimple::step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions) {
    std::bitset<32> bit_set;
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

int MultiSamplingSimple::step_from_to_buchi (int paraent_ba, std::vector<std::vector<double>> all_new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions){
    std::bitset<32> bit_set;
    int buchi_num = paraent_ba;
    for (int k = 0; k < all_new_sample_state.size(); k++){
        for (int i = 0; i < all_interest_regions.size(); i++) {
            if (if_in_region(all_new_sample_state[k], all_interest_regions.find(i)->second)) {
                bit_set.set(all_interest_regions.find(i)->second.get_region_interest());
                break;
            }
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


void MultiSamplingSimple::read_formula(std::string ltl_formula, std::vector<std::string> buchi_regions, std::vector<int> indep_set) {
    SpotHoaInterpreter ltl2ba_lib;
    ba_ = ltl2ba_lib.GetBuchi(ltl_formula, buchi_regions);
    buchi_post(ba_, indep_set);
}
void MultiSamplingSimple::init_workspace(double work_space_size_x,double work_space_size_y) {
    work_space_size_x_ = work_space_size_x;
    work_space_size_y_ = work_space_size_y;
}

void MultiSamplingSimple::init_parameter(double EPSILON, double RADIUS){
    EPSILON_ = EPSILON;
    RADIUS_ = RADIUS;
}

std::vector<double> MultiSamplingSimple::sample_state(std::vector<int> ba_act) {
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
    std::vector<double> sampled_position = {new_node_x, new_node_y}; 
    return sampled_position;
}

std::vector<std::vector<double>> MultiSamplingSimple::multi_sample_state (std::vector<int> ba_act){
    std::vector<std::vector<double>> multi_sampled_position;
    for (int i = 0; i < num_agent_; i++){
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
        std::vector<double> sampled_position = {new_node_x, new_node_y}; 
        multi_sampled_position.push_back(sampled_position);
    }
    return multi_sampled_position;
}

void MultiSamplingSimple::set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, int interest_id) {
    Region interest_region;
    interest_region.set_position(position_x, position_y);
    interest_region.set_region_interest(interest_id);
    all_interest_regions_[interest_id] = interest_region;
}

void MultiSamplingSimple::set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y){
    Region obstacle;
    obstacle.set_position(position_x, position_y);
    all_obstacles_.push_back(obstacle);
}

void MultiSamplingSimple::set_init_state(std::vector<std::vector<double>> init_all_states) {
    all_space_.set_space(ba_.state_num);
    int init_ba = ba_.init_state_idx;
    int acc_ba = ba_.acc_state_idx.front();
    MultiSampleNode init_node;
    init_node.set_id(0);
    init_node.set_all_states(init_all_states);
    init_node.set_ba(ba_.init_state_idx);
    init_node.set_cost(0.0);
    all_space_.insert_sample(init_node, init_ba);
}

void MultiSamplingSimple::start_sampling(int iteration) {
    bool find_path = false;
    uint64_t first_acc_state_id;
    lcm::LCM lcm;
    for (int i = 0; i < iteration; i++) {
        std::vector<int> ba_act = sample_from_ba(ba_, all_space_);
        std::vector<std::vector<double>> sampled_position = multi_sample_state(ba_act); 
        MultiSampleNode parent_sample = all_space_.get_sub_space(ba_act[0]).get_parent(sampled_position);
        std::vector<std::vector<double>> new_sample_state = step_from_to(parent_sample, sampled_position);
        if (Region::collision_check_multi_simple(parent_sample.get_all_states(), new_sample_state, all_obstacles_) ){
            continue;
        }
        // std::cout << "~~~~~~" << std::endl;

        int new_ba = step_from_to_buchi(parent_sample.get_ba(), new_sample_state, ba_, all_interest_regions_);
        MultiSampleNode &chosen_parent_sample = all_space_.get_sub_space(parent_sample.get_ba()).rechoose_parent(parent_sample, new_sample_state, all_obstacles_, RADIUS_);
        // MultiSampleNode &chosen_parent_sample = parent_sample;
        MultiSampleNode new_node;
        uint64_t new_id = all_space_.get_sub_space(new_ba).num_samples();
        new_node.set_id(new_id);
        new_node.set_ba(new_ba);

        chosen_parent_sample.add_children_id(std::make_pair(new_ba, new_id));

        new_node.set_all_states(new_sample_state);
        new_node.set_cost(chosen_parent_sample.get_cost() + get_dist(chosen_parent_sample.get_all_states(), new_sample_state));
        new_node.set_parent_ba(chosen_parent_sample.get_ba());
        new_node.set_parent_id(chosen_parent_sample.get_id());
        all_space_.insert_sample(new_node, new_ba);

        all_space_.rewire(new_id, new_ba, all_obstacles_, RADIUS_);

        // // Vis for debug
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

std::vector<std::vector<std::vector<double>>> MultiSamplingSimple::get_path() {
    bool find_path = false;
    int current_ba = ba_.acc_state_idx.front();
    int init_ba = ba_.init_state_idx;

    if (all_space_.get_sub_space(current_ba).num_samples() > 0) {
        find_path = true;
    }
    else {
        return path_;
    }
    
    double total_cost = INT_MAX;
    uint64_t current_id = 1;
    if (find_path) {
        MultiSampleNode min_cost_sample = all_space_.get_sub_space(ba_.acc_state_idx.front()).get_min_cost_sample();
        std::cout << "Path cost is: " << min_cost_sample.get_cost() << std::endl;
        current_id = min_cost_sample.get_id();
        while (current_ba != init_ba || current_id != 0) {
            
            MultiSampleNode current_node = all_space_.get_sub_space(current_ba).get_sample(current_id);
            
            std::vector<std::vector<double>> path_node = current_node.get_all_states();
            path_.push_back(path_node);
            current_ba = current_node.get_parent_ba();
            current_id = current_node.get_parent_id();
            
        }
    }
    std::reverse(path_.begin(), path_.end());
    return path_;
}