#include "sampling/sample_space.h"
#include "trans_sys/spot_hoa_interpreter.h"
// #include "trans_sys/buchi_automaton.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"
// #include "trajectory/dubins_steer.h"
#include "trajectory/dubins_path.h"

using namespace acel;
typedef struct{
    double x;
    double y;
    double t;
}WayPoint;

class LTL_SamplingDubins {
public:
    LTL_SamplingDubins(){};
    ~LTL_SamplingDubins(){};
private:
    double work_space_size_x_;
    double work_space_size_y_;
    std::string ltl_formula_;
    double EPSILON_;
    double RADIUS_;
    double min_radius_;
    double ground_speed_;
    double time_step_;
    double path_step_;
    std::vector<std::vector<double>> path_;
    std::vector<WayPoint> path_way_points_;

    BAStruct ba_;
    SampleSpace all_space_;
    std::map<int, Region> all_interest_regions_;
    std::vector<Region> all_obstacles_;
    
    double get_dist_dubins(std::vector<double> states_1, std::vector<double> states_2, double min_radius);
    double fRand(double fMin, double fMax);
    std::vector<int> sample_from_ba(BAStruct buchi, SampleSpace &sample_space);
    void buchi_post (BAStruct &ba, std::vector<int> indep_set);
    bool if_in_region (std::vector<double> state, Region region);
    std::vector<double> step_from_to (SampleNode parent_sample, std::vector<double> sampled_state, DubinsPath::PathData& dubins_steer_data, double EPSILON);
    int step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions);
    std::vector<double> sample_state (std::vector<int> ba_act);
    
public:
    void read_formula(std::string ltl_formula, std::vector<std::string> buchi_regions, std::vector<int> indep_set);
    void init_workspace(double work_space_size_x,double work_space_size_y);
    void init_parameter(double EPSILON, double RADIUS, double min_radius, double ground_speed, double time_step);
    void set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, int interest_id);
    void set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y);
    void set_init_state(std::vector<double> init_state);
    void start_sampling(int iteration);
    std::vector<std::vector<double>> get_path();
    std::vector<WayPoint> get_waypoints();
    double get_path_length();
};