#include <algorithm>

#include "sampling/sample_node.h"

SampleNode::SampleNode(uint64_t id, std::pair<double, double> pos){
    id_ = id;
    position_ = pos;
}

std::pair<double, double> SampleNode::get_postion(){
    return position_;
}

void SampleNode::set_position(std::pair<double, double> pos){
    position_ = pos;
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