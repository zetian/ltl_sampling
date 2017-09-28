// #ifndef SRC_SAMPLE_NODE_H_
// #define SRC_SAMPLE_NODE_H_
#include <map>
#include <vector>
#include <utility>

class SampleNode {
public:
    SampleNode(){};
    SampleNode(uint64_t id, std::vector<double> states);
    ~SampleNode(){};

private:
    std::vector<double> state_;
    uint64_t id_;
    int ba_state_;
    double cost_;
    uint64_t parent_id_;
    int parent_ba_;
    std::vector<std::pair<int, uint64_t>> children_;

public:
    std::vector<double> get_state();
    void set_state(std::vector<double> state);

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

    std::vector<std::pair<int, uint64_t>> get_children_id();
    void set_children_id(std::vector<std::pair<int, uint64_t>> children);

};


class SubSampleSpace {
public:
    SubSampleSpace(){};
    ~SubSampleSpace(){};
private:
    std::vector<SampleNode> sample_nodes_;
    int ba_state_;
    std::map<int, SampleNode> sample_node_id_map_; 
    double get_dist(std::vector<double> states_1, std::vector<double> states_2);
public:
    void insert_sample(SampleNode new_sample);
    SampleNode get_sample(uint64_t id);
    SampleNode get_min_cost_sample();
    int num_samples();
    int get_ba_state();
    SampleNode& get_parent(std::vector<double> state);
    SampleNode& rechoose_parent(SampleNode parent_sample, std::vector<double> state, double RADIUS);

    void rewire(SampleNode new_sample);

};