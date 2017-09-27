#include "sampling/sample_space.h"

SampleSpace::SampleSpace(int num_ba) {
    num_ba_ = num_ba;
    for (int i = 0; i < num_ba; i++) {
        SubSampleSpace new_sub_space;
        sub_sample_space_.push_back(new_sub_space);
        sample_space_ltl_map_[i] = new_sub_space;
    }
}

SubSampleSpace& SampleSpace::get_sub_space(int num_ba) {
    return sample_space_ltl_map_.find(num_ba)->second;
    // return sample_space_ltl_map_[num_ba];
}

void SampleSpace::set_space(int num_ba) {
    num_ba_ = num_ba;
    for (int i = 0; i < num_ba; i++) {
        SubSampleSpace new_sub_space;
        sub_sample_space_.push_back(new_sub_space);
        sample_space_ltl_map_[i] = new_sub_space;
    }
}

void SampleSpace::insert_sample(SampleNode new_sample, int sub_space_id) {
    sample_space_ltl_map_.find(sub_space_id)->second.insert_sample(new_sample);
}

uint64_t SampleSpace::total_sample_num() {
    uint64_t total_num = 0;
    for (int i = 0; i < num_ba_; i++) {
        total_num = total_num + sample_space_ltl_map_[i].num_samples();
    }
    return total_num;
}