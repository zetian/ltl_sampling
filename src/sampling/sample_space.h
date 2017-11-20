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
    double step_ = 0.1;
    double get_dist(std::vector<double> states_1, std::vector<double> states_2);
    double get_dist_dubins(std::vector<double> states_1, std::vector<double> states_2, double min_radius);
public:
    void set_space(int num_ba);
    SubSampleSpace& get_sub_space(int num_ba);
    void insert_sample(SampleNode new_sample, int sub_space_id);
    uint64_t total_sample_num();
    void rewire(uint64_t rewire_sample_id, int rewire_sample_ba, std::vector<Region> obstacles, double RADIUS);
    void rewire_dubins(uint64_t rewire_sample_id, int rewire_sample_ba, std::vector<Region> obstacles, double work_space_size_x, double work_space_size_y, double RADIUS, double min_radius, double path_step, double collision_check_rate);
};