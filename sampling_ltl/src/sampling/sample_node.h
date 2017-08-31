// #ifndef SRC_SAMPLE_NODE_H_
// #define SRC_SAMPLE_NODE_H_

#include <vector>

class SampleNode {
public:
    SampleNode();
    SampleNode(uint64_t id, std::vector<double> states);
    ~SampleNode();

private:
    std::vector<double> states_;
    uint64_t id_;
    int ba_state_;
    double cost_;
    uint64_t parent_id_;
    int parent_ba_;
    std::vector<uint64_t> children_;

public:
    std::vector<double> get_states();
    void set_states(std::vector<double> states);

    uint64_t get_id();
    void set_id(uint64_t id);

    int get_ba();
    void set_ba(int ba_id);

    double get_cost();
    void set_cost(double cost);

    uint64_t get_parent_id();
    void set_parent_id(uint64_t parent_id);

    int get_parent_ba();
    void set_parent_ba(int parent_ba);

    std::vector<uint64_t> get_children_id();
    void set_children_id(std::vector<uint64_t> children);


};