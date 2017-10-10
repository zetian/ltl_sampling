#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <cmath>

#include "trans_sys/cbba_Agent_sampling.h"
#include "trans_sys/ltl_formula_sampling.h"
#include "sampling/ltl_sampling_dubins.h"

using namespace acel;

class CBBA_sampling {
public:
    CBBA_sampling();
    ~CBBA_sampling(){};
private:
    // The number of agents
    // const int N = 4;
    // The number of tasks
    // const int M = 6;
    // The number of helpers required
    // const int N_helper = 2;

    double eps = 0;
    // int max_bundle_length = M;
    int iteration_cbba_;

    double work_space_size_x_;
    double work_space_size_y_;
    double EPSILON_;
    double RADIUS_;
    // double radius_L_;
    // double radius_R_;
    LTLFormula Global_LTL_;
    std::vector<std::string> buchi_regions_;
    std::vector<int> indep_set_;

    int num_tasks_;
    int max_bundle_length;
    std::vector<Region> all_interest_regions_;
    std::map<int, Region> all_interest_regions_map_;
    std::vector<Region> all_obstacles_;
    int num_agent_;
    std::vector<cbba_Agent> all_agent_;

public:
    void set_global_ltl(LTLFormula formula);
    void set_buchi_regions(std::vector<std::string> buchi_regions);
    void set_indep_set(std::vector<int> indep_set);
    void set_interest_region(std::pair <double, double> position_x, std::pair <double, double> position_y, int interest_id);
    void set_obstacle(std::pair <double, double> position_x, std::pair <double, double> position_y);
    void init_workspace(double work_space_size_x, double work_space_size_y);
    void init_parameter(int iteration_cbba, double EPSILON, double RADIUS);
    void add_agent(cbba_Agent agent);

    std::vector<int> award_update_for_sampling(cbba_Agent& agent);
    double path_length_calculation(std::string ltl_new, cbba_Agent& agent);
    void neighbor_finder();
	void communicate();
	void available_tasks_finder(cbba_Agent& agent_sig);
	int desired_task_finder(cbba_Agent& agent_sig);
	void bundle_remove();
	void path_remove();
	void bundle_add_for_sampling();
	bool success_checker();
    void start_cbba();
    void get_solution();

};
