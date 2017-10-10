/*
 * ltl_formula_sampling.cpp
 *
 *  Created on: Oct 4, 2017
 *      Author: jfang
 */

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>


// self-define library
#include "trans_sys/ltl_formula_sampling.h"


using namespace acel;

std::vector<std::vector<std::string>> LTLDecomposition::ObtainBuchiRegion(std::vector<std::string> expression){

	std::string IntegerNum = "p0123456789";
    std::size_t found;

    std::vector<std::vector<std::string>> BuchiRegions = {};

    for(int i = 0; i < expression.size(); i++){
    	std::vector<std::size_t> position_p = {0};
        std::vector<std::string> buchi_regions;

        while (position_p.back() != std::string::npos)
        	position_p.push_back(expression[i].find("p",position_p.back()+1,1));

        // Remove the first and the last element in the position vector
        position_p.erase(position_p.begin());
        position_p.pop_back();

        buchi_regions.clear();
        for (int k = 0; k < position_p.size();k++){
        	int j = 0;
            std::string sub_sub_LTL_expression;

            // Obtain the sub_sub_LTL_expression
            if (k != position_p.size()-1)
            	sub_sub_LTL_expression = expression[i].substr(position_p[k],position_p[k+1]-position_p[k]-1);
            else
            	sub_sub_LTL_expression = expression[i].substr(position_p[k]);

            // Only work on the sub_sub_LTL_expression
            while (found != std::string::npos){
            	found = IntegerNum.find(sub_sub_LTL_expression[j]);
            	j++;
            }

            buchi_regions.push_back(sub_sub_LTL_expression.substr(0,j-1));
            found = {0};
        }

        BuchiRegions.push_back(buchi_regions);
    }


    //Print out the buchi_regions for the sub_task
    for (int i = 0; i < BuchiRegions.size();i++){
    	for(int j = 0; j < BuchiRegions[i].size();j++)
    		std::cout << BuchiRegions[i][j] << ' ';
    	std::cout << '\n';
    }

    return BuchiRegions;
}

void LTLDecomposition::get_safety_properties(LTLFormula& formula, std::string safty) {
	formula.LTL_expression_Safety = safty;
}

void LTLDecomposition::get_liveness_properties(LTLFormula& formula, std::string liveness) {
	formula.LTL_expression_Liveness = liveness;
}

LTLFormula LTLDecomposition::GlobalLTLDecomposition(LTLFormula formula){

	/*** Enter the global LTL specification ***/
	// std::cout << "Please Enter the Safety Properties: " << '\n';
	// std::getline(std::cin,formula.LTL_expression_Safety);

	// std::cout << "Please Enter the Liveness Properties: " << '\n';
	// std::getline(std::cin,formula.LTL_expression_Liveness);
	
	std::vector<std::string> sub_expression_Liveness = LTLDecomposition::Decomposition(formula.LTL_expression_Liveness);

	std::vector<std::vector<std::string>> sub_buchi_regions_total = LTLDecomposition::ObtainBuchiRegion(sub_expression_Liveness);
	// std::cout << sub_buchi_regions_total[0][0] << std::endl;
	for (auto idx = sub_buchi_regions_total.begin(); idx != sub_buchi_regions_total.end(); idx++){
		
		// for (auto idx1 = formula.task_info.begin();idx1 != formula.task_info.end(); idx1++){
			
		// 	if ((*idx)[0] == (*idx1).LTL_Target_ID)
				formula.sub_buchi_regions.sub_buchi_regions_Liveness.push_back(*idx);
		// 	else
		// 		continue;
		// }
	}
		
	
	// Construct sub_expression for independent tasks
	// Expression of Independent: Liveness
	for (int idx = 0; idx < formula.sub_buchi_regions.sub_buchi_regions_Liveness.size(); idx++)
		for (auto it1 = sub_expression_Liveness.begin();it1 != sub_expression_Liveness.end(); it1++)
			if((*it1).find(formula.sub_buchi_regions.sub_buchi_regions_Liveness[idx][0]) != std::string::npos)
				formula.LTL_expression.sub_LTL_expression_Liveness.push_back(*it1);
			else
				continue;


	if (!formula.LTL_expression_Safety.empty()){
		formula.LTL_expression_Safety.append(" && ");
	// Expression of Independent: Safety + Liveness
	for(int i = 0; i < formula.LTL_expression.sub_LTL_expression_Liveness.size(); i++)
		formula.LTL_expression.sub_LTL_expression.push_back(formula.LTL_expression_Safety+formula.LTL_expression.sub_LTL_expression_Liveness[i]);
	}
		else{
	for(int i = 0; i < formula.LTL_expression.sub_LTL_expression_Liveness.size(); i++)
		formula.LTL_expression.sub_LTL_expression.push_back(formula.LTL_expression.sub_LTL_expression_Liveness[i]);
	}
	// Update the number of Independent and Dependent tasks
	formula.Num_Tasks = formula.LTL_expression.sub_LTL_expression.size();


	/*** Test: Dependent and Independent ***/
	std::cout << "Expression of Independent " << std::endl;
	for (auto it = formula.LTL_expression.sub_LTL_expression.begin(); it != formula.LTL_expression.sub_LTL_expression.end(); it++)
		std::cout << (*it) << std::endl;

//	std::cout << "Expression of Dependent " << std::endl;
//	for (auto it = formula.LTL_expression.sub_LTL_expression_De.begin(); it != formula.LTL_expression.sub_LTL_expression_De.end(); it++)
//		std::cout << *it << std::endl;
//
//	std::cout << "Dependent Tasks are:" << std::endl;
//	for (auto it = formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.begin(); it != formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.end(); it++){
//		for (auto it1 = (*it).begin(); it1 != (*it).end(); it1++)
//			std::cout << *it1 << " ";
//	    std::cout << std::endl;
//	}
//
	std::cout << "Independent Tasks are:" << std::endl;
	for (auto it = formula.sub_buchi_regions.sub_buchi_regions_Liveness.begin(); it != formula.sub_buchi_regions.sub_buchi_regions_Liveness.end(); it++){
		for (auto it1 = (*it).begin(); it1 != (*it).end(); it1++)
			std::cout << *it1 << " ";
	    std::cout << std::endl;
	}
	std::cout << "Total Tasks are " << std::endl;
	for (auto it = formula.LTL_expression.sub_LTL_expression_Liveness.begin(); it != formula.LTL_expression.sub_LTL_expression_Liveness.end(); it++)
		std::cout << *it << std::endl;

	return formula;
}

std::vector<std::string> LTLDecomposition::Decomposition(std::string expression){

	std::vector<std::string> sub_expression;
	sub_expression.clear();

	std::vector<std::size_t> position = {0};

	while(position.back() != std::string::npos)
	{
		position.push_back(expression.find(") && (",position.back()+1,6));
		//std::cout << "The position is :" << position.back() << std:: endl;
	}

	// Remove the first and the last element in the position vector
	//position.erase(position.begin());
	position.pop_back();

	// Decompose the Liveness Property
	//std::string copy_sub_LTL;
	for (std::vector<std::size_t>::iterator it = position.begin();it != position.end();it++)
	{
		if (it == position.begin())
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr(*it,*(it+1)-(*it) +1);
			sub_expression.push_back(expression.substr(*it,*(it+1)-(*it) +1));
			continue;
		}
		else if(it == position.end())
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr(*it);
			sub_expression.push_back(expression.substr(*it));
			continue;
		}
		else
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr((*it)+5,*(it+1)-((*it)+5)+1);
			sub_expression.push_back(expression.substr((*it)+5,*(it+1)-((*it)+5)+1));
			continue;

		}
	}

	for (int i = 0; i < sub_expression.size();i++)
        std::cout << sub_expression[i] << std::endl;

	return sub_expression;
}

std::string LTLDecomposition::subtask_recreator(std::vector<int> bundle,LTLFormula formula){
	std::string SubtaskFromBundle;
	std::string sub_task_bundled;
	sub_task_bundled.clear();
	SubtaskFromBundle.clear();


	if(bundle.size() == 1){
			std::cout << "There is no need to recreate the subtask" << std::endl;
			// std::cout << "Liveness~@#!@#!@#!@#!: " << formula.LTL_expression.sub_LTL_expression_Liveness.size() << std::endl;
			// std::cout << "bundle~@#!@#!@#!@#!: " << bundle[0] << std::endl;
			SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness[bundle[0]];
			// std::cout << "~========================DEBUG after=============================" << std::endl;
	}

	else if(bundle.size() > 1){
		for (int j  = bundle.size()-1; j >= 0;j--){
			if (j == bundle.size()-1 )
				SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness[bundle[j]];
			else{

				if (formula.sub_buchi_regions.sub_buchi_regions_Liveness[bundle[j]].size() == 1)
					SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness[bundle[j]].front() + " && " + SubtaskFromBundle + ")";
				else {
					for (int it_sub_task_idx = formula.sub_buchi_regions.sub_buchi_regions_Liveness[bundle[j]].size()-1;it_sub_task_idx >= 0 ; it_sub_task_idx--)
						SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness[bundle[j]][it_sub_task_idx] + " && " + SubtaskFromBundle + ")";
					}
					//SubtaskFromBundle = "( " + sub_LTL_expression_Liveness[bundle[j]] + " && " + SubtaskFromBundle + ")";
			}
		}
	}
	else
		std::cout << "No task in the bundle" <<std::endl;

	if (!formula.LTL_expression_Safety.empty())
		sub_task_bundled = formula.LTL_expression_Safety + SubtaskFromBundle;
	else
		sub_task_bundled = SubtaskFromBundle;

	return sub_task_bundled;
}


