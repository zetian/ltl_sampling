#include <map>
#include <vector>
#include <utility>

class SampleNodeDubins {
public:
    SampleNodeDubins(){};
    SampleNodeDubins(uint64_t id, std::vector<double> states);
    ~SampleNodeDubins(){};

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

    std::vector<std::pair<int, uint64_t>>& get_children_id();
    void set_children_id(std::vector<std::pair<int, uint64_t>> children);
    void add_children_id(std::pair<int, uint64_t> one_children);

};