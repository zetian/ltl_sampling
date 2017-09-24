#include "sampling/region.h"

void Region::set_region_interest(int interest) {
    region_interest_ = interest;
}

void Region::set_position(std::pair<double, double> position_x, std::pair<double, double> position_y) {
    position_x_ = position_x;
    position_y_ = position_y;
}

int Region::get_region_interest() {
    return region_interest_;
}

std::pair<double, double> Region::get_x_position() {
    return position_x_;
}

std::pair<double, double> Region::get_y_position() {
    return position_y_;
}
