/*
 * spot_hoa_interpreter.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: rdu, zzhang
 */

#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <bitset>
#include <cmath>
#include <map>
#include <string>

#include <spot/tl/print.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/neverclaim.hh>
#include <spot/parseaut/public.hh>
#include <spot/twaalgos/postproc.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twa/bddprint.hh>
#include <spot/misc/bddlt.hh>
#include <spot/misc/minato.hh>
#include "trans_sys/spot_hoa_interpreter.h"
// #include "trans_sys/buchi_automaton.h"
#include "task_region.h"

using namespace acel;

SpotHoaInterpreter::SpotHoaInterpreter()
{
	for(int i = 0; i < TaskRegion::max_label_num; i++) {
		TaskRegion region(i);
		alphabet_.insert(std::make_pair(region.GetRegionName(), region.GetRegionBitMap()));
	}

	//	for(auto it = alphabet_.begin(); it != alphabet_.end(); it++)
	//		std::cout<< "key: " << it->first << " value: " << it->second << std::endl;
}

SpotHoaInterpreter::~SpotHoaInterpreter()
{
}

void SpotHoaInterpreter::GenAlphabetSet(std::vector<uint32_t>& ab_set, std::vector<std::string> ltl_states)
{
	ab_set.clear();
	for(int i = 0; i < std::pow(2,ltl_states.size()); i++)
	{
		uint32_t state = 0;
		uint32_t test_bit = 0x01;
		for(int j = 0; j < ltl_states.size(); j++)
		{
			if(i & (test_bit << j))
				state |= alphabet_[ltl_states[j]];
		}
		ab_set.push_back(state);
	}

	//	std::cout<<"size of set: " << std::pow(2,ltl_states.size()) << std::endl;
	//	std::cout<<"calculated set: "<<std::endl;
	//	for(auto it = ab_set.begin(); it != ab_set.end(); it++)
	//		std::cout<< std::bitset<32>(*it) << std::endl;
}

TransCon SpotHoaInterpreter::TranslateTransCon(std::string trans)
{
	TransCon con;

	con.pos_con.clear();
	con.neg_con.clear();

	if(trans.compare("null") != 0)
	{
		if(trans.compare("1") == 0)
		{
			con.pos_con.push_back(0);
			con.neg_con.push_back(0);
		}
		else
		{
			// remove ' ', '\t', '(', ')'
			trans.erase(std::remove(trans.begin(),trans.end(),' '),trans.end());
			trans.erase(std::remove(trans.begin(),trans.end(),'\t'),trans.end());
			trans.erase(std::remove(trans.begin(),trans.end(),'('),trans.end());
			trans.erase(std::remove(trans.begin(),trans.end(),')'),trans.end());
			std::stringstream trans_stream(trans);

			std::vector<std::string> or_con;
			std::string or_str;
			while (std::getline(trans_stream, or_str, '|'))
			{
				// remove space and '\t'
				or_str.erase(std::remove(or_str.begin(),or_str.end(),' '),or_str.end());
				or_str.erase(std::remove(or_str.begin(),or_str.end(),'\t'),or_str.end());

				if(!or_str.empty())
					or_con.push_back(or_str);
			}
			con.pos_con.resize(or_con.size());
			con.neg_con.resize(or_con.size());
			int con_set_idx = 0;
			for(auto it = or_con.begin(); it != or_con.end(); it++)
			{
				std::stringstream trans_and_stream((*it));

				std::vector<std::string> and_con;
				std::string and_str;
				while (std::getline(trans_and_stream, and_str, '&'))
				{
					if(!and_str.empty())
						and_con.push_back(and_str);
				}

				// no and condition, only one state
				for(auto it = and_con.begin(); it != and_con.end(); it++)
				{
					std::string str = (*it);

					std::size_t found = str.find("!");
					if (found!=std::string::npos) {
						str.erase(std::remove(str.begin(),str.end(),'!'),str.end());
						con.neg_con[con_set_idx] |= alphabet_[str];
					}
					else {
						con.pos_con[con_set_idx] |= alphabet_[str];
					}
				}
				con_set_idx++;
//				for(auto it = and_con.begin(); it != and_con.end(); it++)
//					std::cout << (*it) << " , ";
//				std::cout << std::endl;
			}
		}
	}

	//	for(int i = 0; i < con.pos_con.size(); i++)
	//		std::cout << "pos con: " << std::bitset<32>(con.pos_con[i]) << " , neg_con: " << std::bitset<32>(con.neg_con[i]) << std::endl;

	return con;
}

std::vector<uint32_t> SpotHoaInterpreter::GetTransCon(TransCon con, std::vector<uint32_t>& ab_set)
{
	std::vector<uint32_t> indices;

	//	std::cout << "pos con size: " << con.pos_con.size() << " , ab_set size: " << ab_set.size() << std::endl;
	for(int i = 0; i < con.pos_con.size(); i++)
	{
//		std::cout << "pos con: " << std::bitset<32>(con.pos_con[i]) << std::endl;
//		std::cout << "neg con: " << std::bitset<32>(con.neg_con[i]) << std::endl;
		uint32_t index = 0;
		for(auto it = ab_set.begin(); it != ab_set.end(); it++)
		{
			if((((*it) & con.pos_con[i]) == con.pos_con[i]) && (((*it) & con.neg_con[i]) == 0))
			{
				if (std::find(indices.begin(), indices.end(), index) == indices.end())
				{
					indices.push_back(index);
//					std::cout << "ab_set element: " << std::bitset<32>((*it)) << " ------ added "<< std::endl;
				}
//				else
//					std::cout << "ab_set element: " << std::bitset<32>((*it)) << std::endl;
			}
//			else
//				std::cout << "ab_set element: " << std::bitset<32>((*it)) << std::endl;
			index++;
		}
	}

	return indices;
}

typedef std::map<int, unsigned> ap_map;
typedef std::vector<int> vap_t;
typedef std::map<bdd, std::string, spot::bdd_less_than> sup_map;

BAStruct SpotHoaInterpreter::GetBuchi(std::string ltl_str, std::vector<std::string> ltl_states)
{
	ap_map ap;
	vap_t vap;
	bdd all_ap;
	sup_map sup;

	// call spot to translate ltl formula
	spot::parsed_formula pf = spot::parse_infix_psl(ltl_str);
	spot::translator trans;
	trans.set_type(spot::postprocessor::BA);
	trans.set_pref(spot::postprocessor::Deterministic);
	trans.set_level(spot::postprocessor::High);
	spot::twa_graph_ptr aut = trans.run(pf.f);

	unsigned num_states = aut->num_states();
	bdd all = bddtrue;
	for (auto& i: sup)
		all &= bdd_support(i.first);
	all_ap = aut->ap_vars();
	all = all_ap;
	while (all != bddtrue)
	{
		int v = bdd_var(all);
		all = bdd_high(all);
		ap.insert(std::make_pair(v, vap.size()));
		vap.push_back(v);
	}

	std::vector<std::vector<std::string>> trans_table_str(num_states, std::vector<std::string>(num_states));
	//	std::map<spot::formula, int> region_to_id;
	//	for (spot::formula ap: aut->ap())
	//		region_to_id.insert(std::pair<spot::formula, int>(ap, aut->get_dict()->varnum(ap)));
	int final;
	for( int src = 0; src < num_states; src++){
		int idx = 0;
		for(auto& i:aut->out(src)){
			auto acc = i.acc;
			if (acc!=0U){
				final = src;
			}
			std::ostringstream s;
			std::string str;
			spot::bdd_print_formula(s, aut->get_dict(), i.cond);
			str = s.str();
			//			std::cout <<" to state: " << i.dst << ": " << str << std::endl;
			trans_table_str[src][i.dst]=str;
		}
	}
	// start constructing BAStruct
	BAStruct ba;

	// generate alphabet set
	GenAlphabetSet(ba.alphabet_set, ltl_states);

	// copy data from spot
	ba.state_num = aut->num_states();
	ba.init_state_idx = aut->get_init_state_number();
	ba.acc_state_idx.push_back(final);
	ba.trans_table.resize(aut->num_states());
	ba.trans_table_str.resize(aut->num_states());
	ba.trans_con.resize(aut->num_states());
	for(int i = 0; i < aut->num_states(); i++) {
		for(int j = 0; j < aut->num_states(); j++) {
			// save transition strings
			ba.trans_table_str[i].push_back(trans_table_str[i][j]);
			// translate transition strings
			TransCon trans_cons = TranslateTransCon(trans_table_str[i][j]);
			ba.trans_table[i].push_back(trans_cons);
			// generate transition conditions
			std::vector<uint32_t> cnd = GetTransCon(trans_cons, ba.alphabet_set);
			ba.trans_con[i].push_back(cnd);
		}
	}

	// print data
//	std::cout <<"Init buchi state: " << ba.init_state_idx << std::endl;
//	std::cout << " - Transition Table String: " << std::endl;
//	for(auto it = ba.trans_table_str.begin(); it != ba.trans_table_str.end(); it++)
//	{
//		for(auto ite = (*it).begin(); ite != (*it).end(); ite++)
//			std::cout << std::setw(25) << (*ite) << " ";
//		std::cout << std::endl;
//	}
//
//	std::cout << "\nAlphabet Set: " << std::endl;
//	int idx = 0;
//	for(auto it = ba.alphabet_set.begin(); it != ba.alphabet_set.end(); it++) {
//		std::cout<< std::bitset<32>(*it) << " , " << idx << std::endl;
//		idx++;
//	}
//	std::cout << "\nTransition Table: " << std::endl;
//	for(int i = 0; i < aut->num_states(); i++) {
//		for(auto it = ba.trans_con[i].begin(); it != ba.trans_con[i].end(); it++)
//		{
//			std::string str = "";
//			for(auto itc = (*it).begin(); itc != (*it).end(); itc++)
//			{
//				str = str + std::to_string(*itc) + ",";
//			}
//			std::cout << std::setw(25) << str << " ";
//		}
//		std::cout << "   -----   line " << i << std::endl;
//	}

	return ba;
}

