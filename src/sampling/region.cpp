#include <algorithm>
#include <random>
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


bool Region::collision_check_dubins(std::vector<std::vector<double>> traj, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y){
    // int SAMPLE_NUM = 15;
    int SAMPLE_NUM =  traj.size()/2.5;
    // if (traj.size() < SAMPLE_NUM) {
    //     SAMPLE_NUM = traj.size();
    // }
    if (SAMPLE_NUM < 5) {
        SAMPLE_NUM = traj.size();
    }
    std::vector<int> generated_values;
    for (int i = 0; i < SAMPLE_NUM; i++){
        int num = rand() % traj.size();
        // if(std::find(generated_values.begin(), generated_values.end(), num) != generated_values.end()) {
        //     continue;
        // }
        generated_values.push_back(num);
    }
    for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < generated_values.size(); j++){
            double x = traj[generated_values[j]][0];
            double y = traj[generated_values[j]][1];
            if (x < 0 || x > work_space_size_x || y < 0 || y > work_space_size_y) {
                return true;
            }
            if (x > obstacle[i].get_x_position().first && x < obstacle[i].get_x_position().second && y > obstacle[i].get_y_position().first && y < obstacle[i].get_y_position().second){
                return true;
            }
        }
    }
    
    return false;
}

bool Region::collision_check_simple(std::vector<double> state_s, std::vector<double> state_f, std::vector<Region> obstacle){
    int SAMPLE_NUM = 15;
    std::vector<double> generated_values;
    for(int i = 0; i < SAMPLE_NUM; i++) {
        std::uniform_real_distribution<double> dist(0, 1);
        std::mt19937 rng;
        rng.seed(std::random_device{}());
        double num = dist(rng);
        generated_values.push_back(num);
    }

    for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < SAMPLE_NUM; j++) {
            double x = (state_f[0] - state_s[0])*generated_values[j] + state_s[0];
            double y = (state_s[1] - state_f[1])/(state_s[0] - state_f[0])*(x - state_s[0]) + state_s[1];
            if (x > obstacle[i].get_x_position().first && x < obstacle[i].get_x_position().second && y > obstacle[i].get_y_position().first && y < obstacle[i].get_y_position().second){
                return true;
            }
        }
    }
    return false;
}
bool Region::collision_check_multi_simple(std::vector<std::vector<double>> state_s, std::vector<std::vector<double>> state_f, std::vector<Region> obstacle){
    int SAMPLE_NUM = 15;
    std::vector<double> generated_values;
    for(int i = 0; i < SAMPLE_NUM; i++) {
        std::uniform_real_distribution<double> dist(0, 1);
        std::mt19937 rng;
        rng.seed(std::random_device{}());
        double num = dist(rng);
        generated_values.push_back(num);
    }

    for (int k = 0; k < state_s.size(); k++){
        for (int i = 0; i < obstacle.size(); i++) {
            for (int j = 0; j < SAMPLE_NUM; j++) {
                double x = (state_f[k][0] - state_s[k][0])*generated_values[j] + state_s[k][0];
                double y = (state_s[k][1] - state_f[k][1])/(state_s[k][0] - state_f[k][0])*(x - state_s[k][0]) + state_s[k][1];
                if (x > obstacle[i].get_x_position().first && x < obstacle[i].get_x_position().second && y > obstacle[i].get_y_position().first && y < obstacle[i].get_y_position().second){
                    return true;
                }
            }
        }
    }
    return false;
}

bool Region::collision_check_multi_dubins(std::vector<std::vector<std::vector<double>>> multi_traj, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y){
    int SAMPLE_NUM =  multi_traj.size()/2.5;
    // if (traj.size() < SAMPLE_NUM) {
    //     SAMPLE_NUM = traj.size();
    // }
    if (SAMPLE_NUM < 5) {
        SAMPLE_NUM = multi_traj.size();
    }
    std::vector<int> generated_values;
    for (int i = 0; i < SAMPLE_NUM; i++){
        int num = rand() % multi_traj.size();
        // if(std::find(generated_values.begin(), generated_values.end(), num) != generated_values.end()) {
        //     continue;
        // }
        generated_values.push_back(num);
    }
    for (int k = 0; k < multi_traj[0].size(); k++){
        for (int i = 0; i < obstacle.size(); i++) {
            for (int j = 0; j < generated_values.size(); j++){
                double x = multi_traj[generated_values[j]][k][0];
                double y = multi_traj[generated_values[j]][k][1];
                if (x < 0 || x > work_space_size_x || y < 0 || y > work_space_size_y) {
                    return true;
                }
                if (x > obstacle[i].get_x_position().first && x < obstacle[i].get_x_position().second && y > obstacle[i].get_y_position().first && y < obstacle[i].get_y_position().second){
                    return true;
                }
            }
        }
    }
    return false;
}

bool Region::collision_check_multi_dubins(std::vector<DubinsPath::PathData> multi_dubins_steer_data, std::vector<Region> obstacle, double work_space_size_x, double work_space_size_y){
    for (int k = 0; k < multi_dubins_steer_data.size(); k++){
        std::vector<std::vector<double>> multi_traj = multi_dubins_steer_data[k].traj_point_wise;
        int SAMPLE_NUM =  multi_traj.size()/2.5;
        // if (traj.size() < SAMPLE_NUM) {
        //     SAMPLE_NUM = traj.size();
        // }
        if (SAMPLE_NUM < 5) {
            SAMPLE_NUM = multi_traj.size();
        }
        std::vector<int> generated_values;
        for (int i = 0; i < SAMPLE_NUM; i++){
            int num = rand() % multi_traj.size();
            // if(std::find(generated_values.begin(), generated_values.end(), num) != generated_values.end()) {
            //     continue;
            // }
            generated_values.push_back(num);
        }
    
        for (int i = 0; i < obstacle.size(); i++) {
            for (int j = 0; j < generated_values.size(); j++){
                double x = multi_traj[generated_values[j]][0];
                double y = multi_traj[generated_values[j]][1];
                if (x < 0 || x > work_space_size_x || y < 0 || y > work_space_size_y) {
                    return true;
                }
                if (x > obstacle[i].get_x_position().first && x < obstacle[i].get_x_position().second && y > obstacle[i].get_y_position().first && y < obstacle[i].get_y_position().second){
                    return true;
                }
            }
        }
    }
    return false;
}
