/*
 * test_buchi.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: rdu
 */

// C++ STL header
#include <stdio.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <ctime>
#include <bitset>

// User-defined header
#include "trans_sys/buchi_automaton.h"

using namespace srcl;
using namespace std;

int main(int argc, char** argv )
{

	std::cout << "Test Buchi Automata:\n" << std::endl;

	//BuchiAutomaton buchi2;

	std::vector<std::string> states1;
	std::vector<std::string> states2;
	std::vector<std::string> states3;
	std::vector<std::string> states4;

	std::string ltl_stm1 = "<> p1";
	states1.push_back("p1");

	std::string ltl_stm2 = "[]<>p0 || <>[]p1";
	states2.push_back("p0");
	states2.push_back("p1");

	std::string ltl_stm3 = "([] p1) && ([] !p2) && (<> p3) && (<> p4)";
	states3.push_back("p1");
	states3.push_back("p2");
	states3.push_back("p3");
	states3.push_back("p4");

	std::string ltl_stm4 = "([] p1) && ([] !p2) && (<> (p3 && <> p4))";
	states4.push_back("p1");
	states4.push_back("p2");
	states4.push_back("p3");
	states4.push_back("p4");

	// std::cout << "\n ------------ test case 1 ------------\n" << std::endl;
	// buchi2.GetBuchi(ltl_stm1);
	// buchi2.GetBuchi(ltl_stm1, states1);
//
//	std::cout << "\n ------------ test case 2 ------------\n" << std::endl;
////	buchi2.GetBuchi(ltl_stm2);
//	buchi2.GetBuchi(ltl_stm2, states2);
//
//	std::cout << "\n ------------ test case 3 ------------\n" << std::endl;
	// buchi2.GetBuchi(ltl_stm3);
	// buchi2.GetBuchi(ltl_stm3, states3);
//
//	std::cout << "\n ------------ test case 4 ------------\n" << std::endl;
////	buchi2.GetBuchi(ltl_stm4);
//	buchi2.GetBuchi(ltl_stm4, states4);

//	std::cout << "\n ------------ test case 5 ------------\n" << std::endl;
//	buchi2.GetBuchi(ltl_stm4);

//	buchi2.TranslateTransCon("(p1 && !p2 && p3) || (!p4)");
//	std::vector<std::string> states;
//	std::vector<uint32_t> ab_set;
//	states.push_back("p1");
//	states.push_back("p2");
//	states.push_back("p3");
//	states.push_back("p4");
//	buchi2.GetAlphabetSet(ab_set, states);
//	for(auto it = ab_set.begin(); it != ab_set.end(); it++)
//			std::cout<< std::bitset<32>(*it) << std::endl;

//	buchi2.GetBuchi(ltl_stm4, states);

	std::cout << "\n ------------ test buchi graph ------------\n" << std::endl;
//	Graph<BuchiState>* buchi_graph = buchi2.CreateBuchiGraph(ltl_stm3,states3);

	//BuchiAutomaton buchi_autaton;
	std::vector<std::string> buchi_regions;
	std::string ltl_formula = "([] p0) && ([] ! p1) && ((<> p2)||(<> p3)||(<> p4))";
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p3");
	buchi_regions.push_back("p4");
//	auto buchi_graph = buchi_autaton.CreateBuchiGraph(ltl_formula,buchi_regions);
//	buchi_autaton.GetBuchi(ltl_formula,buchi_regions);
//	Ltl2baWrapper ltl2ba_lib;
//	BAStruct ba = ltl2ba_lib.GetBuchi(ltl_formula,buchi_regions);

//	delete buchi_graph;

    return 0;
}
