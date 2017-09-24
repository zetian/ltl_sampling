#include <algorithm>
#include <cmath>
#include "sampling/sample_node.h"

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

std::vector<uint64_t> SampleNode::get_children_id(){
    return children_;
}
void SampleNode::set_children_id(std::vector<uint64_t> children){
    children_ = children;
}

double SubSampleSpace::get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[0] - states_2[0], 2));
    return dist;
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

SampleNode SubSampleSpace::get_sample(uint64_t id) {
    return sample_node_id_map_.find(id)->second;
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