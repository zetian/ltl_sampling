/*
 * ltl_formula_sampling.h
 *
 *  Created on: Oct 4, 2017
 *      Author: jfang
 */

#ifndef SRC_TRANS_SYS_LTL_FORMULA_SAMPLING_H_
#define SRC_TRANS_SYS_LTL_FORMULA_SAMPLING_H_

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

namespace acel{

typedef struct
{
	std::string LTL_Target_ID;
	int LTL_ID_;
}Target_Data;


typedef struct
{

    std::vector<std::vector<std::string>> sub_buchi_regions_Safety;
    std::vector<std::vector<std::string>> sub_buchi_regions_Liveness;

}Buchi_Regions;

typedef struct
{

	std::vector<std::string> sub_LTL_expression_Liveness;
	std::vector<std::string> sub_LTL_expression;

}Expression;


struct LTLFormula{

public:
	LTLFormula():
		task_info(){};
	~LTLFormula(){};


public:
	std::vector<std::string> all_buchi_region_;
	std::string LTL_expression_Safety;
	std::string LTL_expression_Liveness;

	Expression LTL_expression;
    Buchi_Regions sub_buchi_regions;

    int Num_Tasks;

    std::vector<Target_Data> task_info;
};

namespace LTLDecomposition
{

LTLFormula GlobalLTLDecomposition(LTLFormula formula);
void get_safety_properties(LTLFormula& formula, std::string safty);
void get_liveness_properties(LTLFormula& formula, std::string liveness);
std::vector<std::string> Decomposition(std::string expression);
std::vector<std::vector<std::string>> ObtainBuchiRegion(std::vector<std::string> expressions);
std::string subtask_recreator(std::vector<int> bundle,LTLFormula formula);

}
}



#endif /* SRC_TRANS_SYS_LTL_FORMULA_SAMPLING_H_ */
