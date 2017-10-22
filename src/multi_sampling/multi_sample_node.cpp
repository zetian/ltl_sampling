#include <algorithm>
#include <cmath>
#include <climits>
#include "multi_sampling/multi_sample_node.h"


MultiSampleNode::MultiSampleNode(uint64_t id){
    id_ = id;
}

std::vector<double> MultiSampleNode::get_state(){
    return state_;
}

std::vector<std::vector<double>> MultiSampleNode::get_all_states(){
    return all_state_;
}

std::map<int, std::vector<double>> MultiSampleNode::get_all_state_map(){
    return all_state_map_;
}

void MultiSampleNode::set_all_states(std::vector<std::vector<double>> all_states){
    all_state_ = all_states;
}

void MultiSampleNode::set_state(std::vector<double> state){
    state_ = state;
}

uint64_t MultiSampleNode::get_id(){
    return id_;
}
void MultiSampleNode::set_id(uint64_t id){
    id_ = id;
}

int MultiSampleNode::get_ba(){
    return ba_state_;
}
void MultiSampleNode::set_ba(int ba_id){
    ba_state_ = ba_id;
}

double MultiSampleNode::get_cost(){
    return cost_;
}
void MultiSampleNode::set_cost(double cost){
    cost_ = cost;
}

uint64_t MultiSampleNode::get_parent_id(){
    return parent_id_;
}
void MultiSampleNode::set_parent_id(uint64_t parent_id){
    parent_id_ = parent_id;
}

int MultiSampleNode::get_parent_ba(){
    return parent_ba_;
}
void MultiSampleNode::set_parent_ba(int parent_ba){
    parent_ba_ = parent_ba;
}

std::vector<std::vector<double>> MultiSampleNode::get_traj() {
    return traj_point_wise_;
}

void MultiSampleNode::set_traj(std::vector<std::vector<double>> traj) {
    traj_point_wise_ = traj;
}



std::vector<std::pair<int, uint64_t>>& MultiSampleNode::get_children_id(){
    return children_;
}
void MultiSampleNode::set_children_id(std::vector<std::pair<int, uint64_t>> children){
    children_ = children;
}

void MultiSampleNode::add_children_id(std::pair<int, uint64_t> one_children) {
    children_.push_back(one_children);
}




double SubSampleSpace::get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[1] - states_2[1], 2));
    return dist;
}

double SubSampleSpace::get_dist(MultiSampleNode multi_sample_1, MultiSampleNode multi_sample_2){
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

double SubSampleSpace::get_dist(std::vector<std::vector<double>> states_1, std::vector<std::vector<double>> states_2){
    if (states_1.size() != states_2.size()){
        return -1;
    }
    double dist = 0;
    for (int i = 0; i < states_1.size(); i++) {
        dist = dist + sqrt(pow(states_1[i][0] - states_2[i][0], 2) + pow(states_1[i][1] - states_2[i][1], 2));
    }
    return dist;
}

double SubSampleSpace::get_dist_dubins(std::vector<double> states_1, std::vector<double> states_2, double radius_L, double radius_R) {
    double min_length = DubinsSteer::GetDubinsCurveLength(states_1, states_2, radius_L, radius_R);
    return min_length;
}

void SubSampleSpace::insert_sample(MultiSampleNode new_sample) {
    sample_nodes_.push_back(new_sample);
    sample_node_id_map_[new_sample.get_id()] = new_sample;
}

int SubSampleSpace::num_samples() {
    return sample_nodes_.size();
}

int SubSampleSpace::get_ba_state() {
    return ba_state_;
}

MultiSampleNode& SubSampleSpace::get_sample(uint64_t id) {
    return sample_node_id_map_.find(id)->second;
}

std::vector<MultiSampleNode>& SubSampleSpace::get_all_samples() {
    return sample_nodes_;
}


MultiSampleNode& SubSampleSpace::get_min_cost_sample() {
    double min_cost = INT_MAX;
    uint64_t min_id = 0;
    for (int i = 0; i < sample_nodes_.size(); i++) {
        // std::vector<double> parent_states = sample_nodes_[i].get_states();
        // double dist = SubSampleSpace::get_dist(parent_states, states);
        if (sample_nodes_[i].get_cost() < min_cost) {
            min_cost = sample_nodes_[i].get_cost();
            min_id = i;
        }
    }
    MultiSampleNode& min_cost_sample = get_sample(min_id);
    return min_cost_sample;
}

MultiSampleNode& SubSampleSpace::get_parent(std::vector<double> state) {
    MultiSampleNode &parent_sample = sample_nodes_.front();
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        if (get_dist(sample_nodes_[i].get_state(), state) < get_dist(parent_sample.get_state(), state)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;
}

MultiSampleNode& SubSampleSpace::get_parent(MultiSampleNode multi_sample){
    MultiSampleNode &parent_sample = sample_nodes_.front();
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        if (get_dist(sample_nodes_[i], multi_sample) < get_dist(parent_sample, multi_sample)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;

}

MultiSampleNode& SubSampleSpace::get_parent(std::vector<std::vector<double>> multi_states){
    MultiSampleNode &parent_sample = sample_nodes_.front();
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        if (get_dist(sample_nodes_[i].get_all_states(), multi_states) < get_dist(parent_sample.get_all_states(), multi_states)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;

}

MultiSampleNode& SubSampleSpace::get_parent_dubins(std::vector<double> state, double radius_L, double radius_R) {
    MultiSampleNode &parent_sample = sample_nodes_.front();
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        if (get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) < get_dist_dubins(parent_sample.get_state(), state, radius_L, radius_R)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;
}

MultiSampleNode& SubSampleSpace::rechoose_parent(MultiSampleNode parent_sample, std::vector<std::vector<double>> all_states, std::vector<Region> obstacles, double RADIUS) {
    MultiSampleNode &new_parent_sample = sample_nodes_.front();
    double new_cost = parent_sample.get_cost() + get_dist(parent_sample.get_all_states(), all_states);
    for (int i = 0; i < sample_nodes_.size(); i++) {
        if (get_dist(sample_nodes_[i].get_all_states(), all_states) < RADIUS &&
            get_dist(sample_nodes_[i].get_all_states(), all_states) + sample_nodes_[i].get_cost() < new_cost) {
            new_parent_sample = sample_nodes_[i];
            if (Region::collision_check_multi_simple(sample_nodes_[i].get_all_states(), all_states, obstacles)) {
                continue;
            }
            new_cost = get_dist(sample_nodes_[i].get_all_states(), all_states) + sample_nodes_[i].get_cost();
        }
    }
    return new_parent_sample;
}

// MultiSampleNode& SubSampleSpace::rechoose_parent_dubins(MultiSampleNode parent_sample, std::vector<double> state, DubinsSteer::SteerData& dubins_steer_data, std::vector<Region> obstacles, double work_space_size_x, double work_space_size_y, double RADIUS, double radius_L, double radius_R) {
//     MultiSampleNode &new_parent_sample = sample_nodes_.front();
//     for (int i = 0; i < sample_nodes_.size(); i++) {
//         double new_cost = parent_sample.get_cost() + get_dist_dubins(parent_sample.get_state(), state, radius_L, radius_R);
//         if (get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) < RADIUS &&
//                 get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) + sample_nodes_[i].get_cost() < new_cost) {
//                 new_parent_sample = sample_nodes_[i];

//                 dubins_steer_data = DubinsSteer::GetDubinsTrajectoryPointWise(sample_nodes_[i].get_state(), state, radius_L, radius_R);
//                 if (Region::collision_check_dubins(dubins_steer_data.traj_point_wise, obstacles, work_space_size_x, work_space_size_y)) {
//                     continue;
//                 }
//                 new_cost = get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) + sample_nodes_[i].get_cost();
//         }
//     }
//     return new_parent_sample;
// }