#include <utility>
class Region {
public:
    Region(){};
    ~Region(){};
private:
    int region_interest_;
    std::pair<double, double> position_x_;
    std::pair<double, double> position_y_;
public:
    void set_region_interest(int interest);
    int get_region_interest();
    void set_position(std::pair<double, double> position_x, std::pair<double, double> position_y);
    std::pair<double, double> get_x_position();
    std::pair<double, double> get_y_position();
};