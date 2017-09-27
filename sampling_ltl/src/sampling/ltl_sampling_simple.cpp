#include <random>
#include <algorithm>
#include <bitset>

#include "sampling/ltl_sampling_simple.h"
double LTL_SamplingSimple::get_dist(std::vector<double> states_1, std::vector<double> states_2) {
    double dist = sqrt(pow(states_1[0] - states_2[0], 2) + pow(states_1[1] - states_2[1], 2));
    return dist;
}

double LTL_SamplingSimple::fRand(double fMin, double fMax)
{
    std::uniform_real_distribution<double> dist(fMin, fMax);
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    return dist(rng);
}

std::vector<int> LTL_SamplingSimple::sample_from_ba(BAStruct buchi, SampleSpace &sample_space)
{
    if (sample_space.total_sample_num() == 0) {
        return std::vector<int>();
    }
    // srand(time(NULL));
    int size_buchi = buchi.state_num;
    std::vector<uint32_t> ba_accept_state = buchi.acc_state_idx;
    int new_ba_sample = rand() % (size_buchi);
    std::cout << "AAAAAA random!!! AAAAAAAAAAA:   " << new_ba_sample << std::endl;
    std::vector<int> new_ba_candidate;
    std::vector<int> temp_candidate;
    std::vector<std::vector<int>> act_q;
    // std::cout << "~~~~~~" << ba_accept_state[0] <<std::endl;
    if (new_ba_sample == ba_accept_state[0])
    {

        for (int i = 0; i < size_buchi; i++)
        {

            if (i != new_ba_sample && !buchi.trans_con[i][new_ba_sample].empty())
            {

                temp_candidate.push_back(i);
                if (sample_space.get_sub_space(i).num_samples() > 0)
                {
                    new_ba_candidate.push_back(i);
                    act_q.push_back({i, new_ba_sample});
                }
            }
        }
    }
    else
    {

        temp_candidate.push_back(new_ba_sample);
        // std::cout << "~~~~~~11" << std::endl;
        // std::cout << "num::::" << sample_space.get_sub_space(new_ba_sample).num_samples() << std::endl;
        if (sample_space.get_sub_space(new_ba_sample).num_samples() > 0)
        {
            // std::cout << "~~~~~~" << std::endl;
            new_ba_candidate.push_back(new_ba_sample);
            act_q.push_back({new_ba_sample, new_ba_sample});
        }
    }
    while (new_ba_candidate.empty())
    {
        std::vector<int> new_temp_candidate;
        for (int i = 0; i < temp_candidate.size(); i++)
        {
            for (int j = 0; j < size_buchi; j++)
            {
                if (j != new_ba_sample && !buchi.trans_con[j][temp_candidate[i]].empty() && std::find(temp_candidate.begin(), temp_candidate.end(), j) == temp_candidate.end())
                {
                    new_temp_candidate.push_back(j);
                    if (sample_space.get_sub_space(j).num_samples() > 0)
                    {
                        // std::cout << "~!!!" << j << std::endl;
                        new_ba_candidate.push_back(j);
                        act_q.push_back({j, temp_candidate[i]});
                    }
                }
            }
        }
        temp_candidate = new_temp_candidate;
        // std::cout << "~~~back" << std::endl;
    }
    // for (auto &i:new_ba_candidate) {
    //     std::cout << i << " ";
    // }
    // srand(time(NULL));
    int r = rand() % (new_ba_candidate.size());
    // std::cout << "random num: " << r << std::endl;
    int new_ba_state = new_ba_candidate[r];
    // std::cout << "new_ba_state: " << new_ba_state << std::endl;
    std::cout << "BBBBBB sampled buchi state:    " << act_q[r][0] << ", and: "<< act_q[r][1] << std::endl;
    return act_q[r];
}

void LTL_SamplingSimple::buchi_post (BAStruct &ba, std::vector<int> indep_set) {
    std::bitset<32> indep_bitset;
    for (auto it = indep_set.begin(); it != indep_set.end(); it++) {
        indep_bitset.set(*it, 1);
    }
    // std::cout << "indep_bitset: " << indep_bitset << std::endl;
    int alphabet_set_size = ba.alphabet_set.size();
    std::vector<int> indep_alphabet;
    for (int i = 0; i < alphabet_set_size; i++) {
        std::bitset<32> check_indep = std::bitset<32>(ba.alphabet_set[i]) & indep_bitset;
        if (check_indep.count() > 1) {
            indep_alphabet.push_back(i);
        }
    }

    // std::cout << "indep_alphabet: " << std::endl;
    // for (auto it = indep_alphabet.begin(); it != indep_alphabet.end(); it++) {
    //     std::cout << *it << " ";
    // }
    // std::cout << std::endl;

    

    for (int i = 0; i < ba.state_num; i++)
    {
        for (auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
        {
            std::vector<int> inter;
            std::set_intersection((*it).begin(), (*it).end(), indep_alphabet.begin(), indep_alphabet.end(), std::back_inserter(inter));
            (*it).erase(std::set_difference((*it).begin(), (*it).end(), inter.begin(), inter.end(), (*it).begin()), (*it).end());

            // std::string str = "";
            // for (auto itc = (*it).begin(); itc != (*it).end(); itc++)
            // {
            //     str = str + std::to_string(*itc) + ",";
            // }
            // std::cout << std::setw(25) << str << " ";
        }
        // std::cout << "   -----   line " << i << std::endl;
    }

}

bool LTL_SamplingSimple::if_in_region (std::vector<double> state, Region region) {
    if (state[0] > region.get_x_position().first && state[0] < region.get_x_position().second && state[1] > region.get_y_position().first && state[1] < region.get_y_position().second) {
        return true;
    }
    else {
        return false;
    }
}

std::vector<double> LTL_SamplingSimple::step_from_to (SampleNode parent_sample, std::vector<double> sampled_state, double EPSILON) {
    if (get_dist(parent_sample.get_state(), sampled_state) < EPSILON) {
        return sampled_state;
    }
    else {
        double theta = atan2(sampled_state[1] - parent_sample.get_state()[1], sampled_state[0] - parent_sample.get_state()[0]);
        std::vector<double> new_state = {parent_sample.get_state()[0] + EPSILON*cos(theta), parent_sample.get_state()[1] + EPSILON*sin(theta)};
        return new_state;
    }
}

int LTL_SamplingSimple::step_from_to_buchi (int paraent_ba, std::vector<double> new_sample_state, BAStruct ba, std::map<int, Region> all_interest_regions) {
    std::bitset<32> bit_set;
    std::cout << "*******in step from to buchi function: ******" << std::endl;
    std::cout << "parent ba: " << paraent_ba << std::endl;
    int buchi_num = paraent_ba;
    for (int i = 0; i < all_interest_regions.size(); i++) {
        if (if_in_region(new_sample_state, all_interest_regions.find(i)->second)) {
            // std::cout << "(step_from_to_buchi function) interested region num: " << all_interest_regions.find(i)->second.get_region_interest() << std::endl;
            bit_set.set(all_interest_regions.find(i)->second.get_region_interest());
            std::cout << "new state is in region: " << all_interest_regions.find(i)->second.get_region_interest() << std::endl;
            break;
        }
    }
    int act = bit_set.to_ullong();
    // std::cout << "(step_from_to_buchi function) paraent buchi state: " << paraent_ba << std::endl;
    for (int i = 0; i < ba.trans_con[paraent_ba].size(); i++) {
        if (std::find(ba.trans_con[paraent_ba][i].begin(), ba.trans_con[paraent_ba][i].end(), act) != ba.trans_con[paraent_ba][i].end()) {
            buchi_num = i;
            break;
        }
    }
    std::cout << "new ba: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) out buchi state: " << buchi_num << std::endl;
    // std::cout << "(step_from_to_buchi function) test bit set: " << bit_set << std::endl;
    // std::cout << "(step_from_to_buchi function) test int: " << bit_set.to_ullong() << std::endl;
    std::cout << "****************************************" << std::endl;
    return buchi_num;
}

void LTL_SamplingSimple::read_formula(std::string ltl_formula, std::vector<std::string> buchi_regions, std::vector<int> indep_set) {
    SpotHoaInterpreter ltl2ba_lib;
    ba_ = ltl2ba_lib.GetBuchi(ltl_formula, buchi_regions);
    buchi_post(ba_, indep_set);
}
void LTL_SamplingSimple::init_workspace(double work_space_size_x,double work_space_size_y) {
    work_space_size_x_ = work_space_size_x;
    work_space_size_y_ = work_space_size_y;
}