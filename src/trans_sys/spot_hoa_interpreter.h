/*
 * ltl2ba_wrapper.h
 *
 *  Created on: Jul 19, 2016
 *      Author: rdu
 */

#ifndef SRC_H2C_SPOT_HOA_INTERPRETER_H_
#define SRC_H2C_SPOT_HOA_INTERPRETER_H_

#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <memory>

#include <spot/tl/parse.hh>

namespace acel{

/// This struct is for internal use only.
typedef struct trans_con{
	std::vector<uint32_t> pos_con;
	std::vector<uint32_t> neg_con;
}TransCon;

/// BAStruct is used to store the data returned by the spot library.
typedef struct {
	uint16_t state_num;
	uint32_t init_state_idx;
	std::vector<uint32_t> acc_state_idx;
	std::vector<std::vector<TransCon>> trans_table;
	std::vector<std::vector<std::string>> trans_table_str;
	std::vector<std::vector<std::vector<uint32_t>>> trans_con;

	std::vector<uint32_t> alphabet_set;
	std::map<spot::formula, int> region_to_id;
}BAStruct;

/// BuchiAutomata class contains functions to create a Buchi graph from LTL statement.
class SpotHoaInterpreter{
public:
	SpotHoaInterpreter();
	~SpotHoaInterpreter();

private:
	std::map<std::string, uint32_t> alphabet_;

private:
	void GenAlphabetSet(std::vector<uint32_t>& ab_set, std::vector<std::string> ltl_states);
	TransCon TranslateTransCon(std::string trans);
	std::vector<uint32_t> GetTransCon(TransCon con, std::vector<uint32_t>& ab_set);

public:
	BAStruct GetBuchi(std::string ltl_str, std::vector<std::string> ltl_states);
};

}

#endif /* SRC_H2C_SPOT_HOA_INTERPRETER_H_ */
