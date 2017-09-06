#include "sampling/sample_space.h"

SampleSpace::SampleSpace(int num_ba) {
    num_ba_ = num_ba;
    for (int i = 0; i < num_ba; i++) {
        SubSampleSpace new_sub_space;
        sub_sample_space_.push_back(new_sub_space);
        sample_space_ltl_map_[i] = new_sub_space;
    }
}