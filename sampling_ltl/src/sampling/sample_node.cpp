#include <algorithm>
#include <cmath>
#include <climits>
#include "sampling/sample_node.h"
#include "trajectory/dubins_steer.h"

SampleNode::SampleNode(uint64_t id, std::vector<double> state){
    id_ = id;
    state_ = state;
}

std::vector<double> SampleNode::get_state(){
    return state_;
}

void SampleNode::set_state(std::vector<double> state){
    state_ = state;
}

uint64_t SampleNode::get_id(){
    return id_;
}
void SampleNode::set_id(uint64_t id){
    id_ = id;
}

int SampleNode::get_ba(){
    return ba_state_;
}
void SampleNode::set_ba(int ba_id){
    ba_state_ = ba_id;
}

double SampleNode::get_cost(){
    return cost_;
}
void SampleNode::set_cost(double cost){
    cost_ = cost;
}

uint64_t SampleNode::get_parent_id(){
    return parent_id_;
}
void SampleNode::set_parent_id(uint64_t parent_id){
    parent_id_ = parent_id;
}

int SampleNode::get_parent_ba(){
    return parent_ba_;
}
void SampleNode::set_parent_ba(int parent_ba){
    parent_ba_ = parent_ba;
}

std::vector<std::vector<double>> SampleNode::get_traj() {
    return traj_point_wise_;
}

void SampleNode::set_traj(std::vector<std::vector<double>> traj) {
    traj_point_wise_ = traj;
}



std::vector<std::pair<int, uint64_t>>& SampleNode::get_children_id(){
    return children_;
}
void SampleNode::set_children_id(std::vector<std::pair<int, uint64_t>> children){
    children_ = children;
}

void SampleNode::add_children_id(std::pair<int, uint64_t> one_children) {
    children_.push_back(one_children);
}




double SubSampleSpace::get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[1] - states_2[1], 2));
    return dist;
}

double SubSampleSpace::get_dist_dubins(std::vector<double> states_1, std::vector<double> states_2, double radius_L, double radius_R) {
    double min_length = DubinsSteer::GetDubinsCurveLength(states_1, states_2, radius_L, radius_R);
    return min_length;
}

void SubSampleSpace::insert_sample(SampleNode new_sample) {
    sample_nodes_.push_back(new_sample);
    sample_node_id_map_[new_sample.get_id()] = new_sample;
}

int SubSampleSpace::num_samples() {
    return sample_nodes_.size();
}

int SubSampleSpace::get_ba_state() {
    return ba_state_;
}

SampleNode& SubSampleSpace::get_sample(uint64_t id) {
    return sample_node_id_map_.find(id)->second;
}

std::vector<SampleNode>& SubSampleSpace::get_all_samples() {
    return sample_nodes_;
}


SampleNode& SubSampleSpace::get_min_cost_sample() {
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
    SampleNode& min_cost_sample = get_sample(min_id);
    return min_cost_sample;
}

SampleNode& SubSampleSpace::get_parent(std::vector<double> state) {
    SampleNode &parent_sample = sample_nodes_.front();
    // if (!sample_nodes_.empty()) {
    //     SampleNode parent_sample = sample_nodes_.front();
    // }
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        // std::vector<double> parent_states = sample_nodes_[i].get_states();
        // double dist = SubSampleSpace::get_dist(parent_states, states);
        if (get_dist(sample_nodes_[i].get_state(), state) < get_dist(parent_sample.get_state(), state)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;
}

SampleNode& SubSampleSpace::get_parent_dubins(std::vector<double> state, double radius_L, double radius_R) {
    SampleNode &parent_sample = sample_nodes_.front();
    // if (!sample_nodes_.empty()) {
    //     SampleNode parent_sample = sample_nodes_.front();
    // }
    
    for (int i = 0; i < sample_nodes_.size(); i++) {
        // std::vector<double> parent_states = sample_nodes_[i].get_states();
        // double dist = SubSampleSpace::get_dist(parent_states, states);
        if (get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) < get_dist_dubins(parent_sample.get_state(), state, radius_L, radius_R)) {
            parent_sample = sample_nodes_[i];
        }
    }
    return parent_sample;
}

SampleNode& SubSampleSpace::rechoose_parent(SampleNode parent_sample, std::vector<double> state, double RADIUS) {
    SampleNode &new_parent_sample = sample_nodes_.front();
    // if (!sample_nodes_.empty()) {
    //     SampleNode parent_sample = sample_nodes_.front();
    // }
    double new_cost = parent_sample.get_cost() + get_dist(parent_sample.get_state(), state);
    for (int i = 0; i < sample_nodes_.size(); i++) {
        // std::vector<double> parent_states = sample_nodes_[i].get_states();
        // double dist = SubSampleSpace::get_dist(parent_states, states);
        if (get_dist(sample_nodes_[i].get_state(), state) < RADIUS &&
            get_dist(sample_nodes_[i].get_state(), state) + sample_nodes_[i].get_cost() < new_cost) {
            new_parent_sample = sample_nodes_[i];
            new_cost = get_dist(sample_nodes_[i].get_state(), state) + sample_nodes_[i].get_cost();
        }
    }
    return new_parent_sample;
}

SampleNode& SubSampleSpace::rechoose_parent_dubins(SampleNode parent_sample, std::vector<double> state, double RADIUS, double radius_L, double radius_R) {
    SampleNode &new_parent_sample = sample_nodes_.front();
    // if (!sample_nodes_.empty()) {
    //     SampleNode parent_sample = sample_nodes_.front();
    // }
        
    for (int i = 0; i < sample_nodes_.size(); i++) {
        // std::vector<double> parent_states = sample_nodes_[i].get_states();
        // double dist = SubSampleSpace::get_dist(parent_states, states);
        // DubinsSteer::SteerData dubins_steer_data_new;
        // DubinsSteer::SteerData dubins_steer_data_old;
        // DubinsSteer::SteerData dubins_steer_data_new = DubinsSteer::GetDubinsTrajectoryPointWise(sample_nodes_[i].get_state(), state, radius_L, radius_R);
        // dubins_steer_data_old = DubinsSteer::GetDubinsTrajectoryPointWise(parent_sample.get_state(), state, radius_L, radius_R);
        double new_cost = parent_sample.get_cost() + get_dist_dubins(parent_sample.get_state(), state, radius_L, radius_R);
        // if (dubins_steer_data_new.traj_length < RADIUS &&
        //     dubins_steer_data_new.traj_length + sample_nodes_[i].get_cost() < new_cost) {
        //         new_cost = dubins_steer_data_new.traj_length + sample_nodes_[i].get_cost();
        //         traj_point_wise = dubins_steer_data_new.traj_point_wise;
        //         new_parent_sample = sample_nodes_[i];
        // }

        if (get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) < RADIUS &&
                get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) + sample_nodes_[i].get_cost() < new_cost) {
                new_parent_sample = sample_nodes_[i];
                new_cost = get_dist_dubins(sample_nodes_[i].get_state(), state, radius_L, radius_R) + sample_nodes_[i].get_cost();
        }

        // if (dubins_steer_data_new.traj_length < RADIUS &&
        //     dubins_steer_data_new.traj_length + sample_nodes_[i].get_cost() < 
        //     parent_sample.get_cost() + dubins_steer_data_old.traj_length) {
        //         // traj_point_wise.clear();
        //         // traj_point_wise = dubins_steer_data_new.traj_point_wise;
        //         new_parent_sample = sample_nodes_[i];
        // }
    }
    return new_parent_sample;
}

// void SubSampleSpace::rewire(uint64_t new_sample_id, double RADIUS) {
//     SampleNode &new_sample = sample_node_id_map_.find(new_sample_id)->second;
//     for (int i = 0; i < sample_nodes_.size(); i++) {
//         if (sample_nodes_[i].get_id() != new_sample.get_parent_id() &&
//             get_dist(sample_nodes_[i].get_state(), new_sample.get_state()) < RADIUS &&
//             get_dist(sample_nodes_[i].get_state(), new_sample.get_state()) + new_sample.get_cost() < 
//             sample_nodes_[i].get_cost() ) {
            
//             SampleNode &rewire_sample = sample_nodes_[i];
//             uint64_t old_parent_id = rewire_sample.get_parent_id();
//             int old_parent_ba = rewire_sample.get_parent_ba();


//         }
//     }

// }