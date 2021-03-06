#include "sampling/sample_space.h"
#include "trans_sys/spot_hoa_interpreter.h"
// #include "trans_sys/buchi_automaton.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

using namespace acel;

class LTL_Sampling3d {
public:
    LTL_Sampling3d(){};
    ~LTL_Sampling3d(){};
private:
    double work_space_size_x_ = 100;
    double work_space_size_y_ = 100;
    double work_space_size_z_ = 100;
    std::string ltl_formula_;
    double EPSILON_ = 8;
    double RADIUS_ = 16;
    std::vector<std::vector<double>> path_;

    BAStruct ba_;
    SampleSpace all_space_;
    std::map<int, Region> all_interest_regions_;
    std::vector<Region> all_obstacles_;
    double get_dist(std::vector<double> states_1, std::vector<double> states_2);
    double fRand(double fMin, double fMax);
    std::vector<int> sample_from_ba(BAStruct buchi, SampleSpace &sample_space);
    void buchi_post (BAStruct &ba, std::vector<int> indep_set);
    bool if_in_region (std::vector<double> state, Region region);
    std::vector<double> step_from_to (SampleNode parent_sample, std::vector<double> sampled_state);
    int step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions);
    std::vector<double> sample_state (std::vector<int> ba_act);
public:
    void read_formula(std::string ltl_formula, std::vector<std::string> buchi_regions, std::vector<int> indep_set);
    void init_workspace(double work_space_size_x, double work_space_size_y, double work_space_size_z);
    void init_parameter(double EPSILON, double RADIUS);
    void set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, std::pair <double, double> position_z, int interest_id);
    void set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y, std::pair <double, double> position_z);
    void set_init_state(std::vector<double> init_state);
    void start_sampling(int iteration);
    std::vector<std::vector<double>> get_path();
};