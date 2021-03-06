#include "trajectory/dubins_path.h"
#include <utility>
#include <vector>
class Region {
public:
    Region(){};
    ~Region(){};
private:
    int region_interest_;
    std::pair<double, double> position_x_;
    std::pair<double, double> position_y_;
    std::pair<double, double> position_z_;
    // bool contains(std::vector<int> container_vector, int num);
public:
    void set_region_interest(int interest);
    int get_region_interest();
    void set_position(std::pair<double, double> position_x, std::pair<double, double> position_y);
    void set_position(std::pair<double, double> position_x, std::pair<double, double> position_y, std::pair<double, double> position_z);
    std::pair<double, double> get_x_position();
    std::pair<double, double> get_y_position();
    std::pair<double, double> get_z_position();
    static bool collision_check_dubins(std::vector<std::vector<double>> traj, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y, double collision_check_rate);
    static bool collision_check_simple(std::vector<double> state_s, std::vector<double> state_f, std::vector<Region> obstacle);
    static bool collision_check_simple_3d(std::vector<double> state_s, std::vector<double> state_f, std::vector<Region> obstacle);
    static bool collision_check_multi_simple(std::vector<std::vector<double>> state_s, std::vector<std::vector<double>> state_f, std::vector<Region> obstacle);
    static bool collision_check_multi_dubins(std::vector<std::vector<std::vector<double>>> multi_traj, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y, double collision_check_rate);
    static bool collision_check_multi_dubins(std::vector<DubinsPath::PathData> multi_dubins_steer_data, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y, double collision_check_rate);

};