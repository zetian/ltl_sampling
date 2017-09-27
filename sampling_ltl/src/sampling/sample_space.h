#include <map>
#include <vector>
#include "sampling/sample_node.h"

class SampleSpace {
public:
    SampleSpace(){};
    SampleSpace(int num_ba);
    ~SampleSpace(){};
private:
    int num_ba_;
    std::vector<SubSampleSpace> sub_sample_space_;
    std::map<int, SubSampleSpace> sample_space_ltl_map_; 
public:
    void set_space(int num_ba);
    SubSampleSpace& get_sub_space(int num_ba);
    void insert_sample(SampleNode new_sample, int sub_space_id);
    uint64_t total_sample_num();

};