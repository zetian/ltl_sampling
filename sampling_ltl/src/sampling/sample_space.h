#include <map>
#include <vector>
#include "sampling/sample_node.h"

class SampleSpace {
public:
    SampleSpace();
    SampleSpace(int num_ba);
    ~SampleSpace();
private:
    int num_ba_;
    std::vector<SubSampleSpace> sub_sample_space_;
    std::map<int, SubSampleSpace> sample_space_ltl_map_; 
// public:
//     void set_sub_space(int num_ba);
};