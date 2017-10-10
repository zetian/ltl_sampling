/*
 * spot_test.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: rdu
 */
//#include <ostream>
//#include <sstream>
//#include <cstring>

//#include <string>
//#include <iostream>
#include <strstream>
#include <map>
#include <spot/tl/parse.hh>
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
//#include <spot/misc/escape.hh>


//#include <spot/twa/twa.hh>
//#include <spot/twa/twagraph.hh>
//#include <spot/twaalgos/hoa.hh>
//#include <spot/twaalgos/reachiter.hh>
//#include <spot/misc/escape.hh>
//#include <spot/misc/bddlt.hh>
//#include <spot/misc/minato.hh>
//#include <spot/twa/formula2bdd.hh>
//#include <spot/tl/formula.hh>
//#include <spot/kripke/fairkripke.hh>


//#include <bddx.h>
//#include <functional>
//#include <spot/twa/twa.hh>
//#include <spot/twa/twagraph.hh>
//#include <spot/twaalgos/hoa.hh>
//#include <spot/twaalgos/reachiter.hh>
//#include <spot/misc/escape.hh>
//#include <spot/misc/bddlt.hh>
//#include <spot/misc/minato.hh>
//#include <spot/twa/formula2bdd.hh>
//#include <spot/tl/formula.hh>
//#include <spot/kripke/fairkripke.hh>



int main()
{
	typedef std::map<int, unsigned> ap_map;
	ap_map ap;
	typedef std::vector<int> vap_t;
	vap_t vap;

	std::vector<bool> common_acc;
	bool has_state_acc;
	bool is_complete;
	bool is_deterministic;
	bool is_colored;
	bool use_implicit_labels;
	bool use_state_labels = true;
	bdd all_ap;

	// Label support: the set of all conditions occurring in the
	// automaton.
	typedef std::map<bdd, std::string, spot::bdd_less_than> sup_map;
	sup_map sup;




	// generate ltl formula
	//	std::string input = "[]<>p0 || <>[]p1";
	//	std::string input = "([] p0) && ([] !p1) && (<> p2) && (<> p3) && (<> p4) && (<> p5)";
//	"([] p0) && ([] ! p1) && ((<> p2)||(<> p3)||(<> p4))";
	std::string input = "([] p0) && ([] ! p1) && ((<> p3)||(<> p2)||(<> p4))";
	spot::parsed_formula pf = spot::parse_infix_psl(input);
	if (pf.format_errors(std::cerr))
		return 1;

	// translate to BA/TGBA
	spot::translator trans;
	trans.set_type(spot::postprocessor::BA);
	trans.set_pref(spot::postprocessor::Deterministic);
	trans.set_level(spot::postprocessor::High);
	spot::twa_graph_ptr aut = trans.run(pf.f);
	unsigned num_states = aut->num_states();
//    bdd all = bddtrue;
//    for (auto& i: sup)
//      all &= bdd_support(i.first);
//    all_ap = aut->ap_vars();
//    if (bdd_exist(all, all_ap) != bddtrue)
//      throw std::runtime_error("print_hoa(): automaton uses "
//                               "unregistered atomic propositions");
//    all = all_ap;
//
//    while (all != bddtrue)
//      {
//        int v = bdd_var(all);
//        all = bdd_high(all);
//        ap.insert(std::make_pair(v, vap.size()));
//        vap.push_back(v);
//      }
//	auto x = aut->out(1);
//	for(auto& i:x){
//		acel::TransCon con;
//
//		con.pos_con.clear();
//		con.neg_con.clear();
////		i = x(0);
////		auto con = i.cond;
//		spot::minato_isop isop(i.cond);
//		bdd cube;
//		bool notfirstor = false;
//		std::ostringstream s;
//		std::string str;
//		while ((cube = isop.next()) != bddfalse)
//		{
//			if (notfirstor)
//				s << " | ";
//			bool notfirstand = false;
//			while (cube != bddtrue)
//			{
//				if (notfirstand)
//					s << '&';
//				else
//					notfirstand = true;
//				bdd h = bdd_high(cube);
//				if (h == bddfalse)
//				{
//					s << '!' << ap[bdd_var(cube)];
//					cube = bdd_low(cube);
//				}
//				else
//				{
//					s << ap[bdd_var(cube)];
//					cube = h;
//				}
//			}
//			notfirstor = true;
//		}
//		str = s.str();
//		std::cout <<" to state: " << i.dst << ": " << str << std::endl;
//	}

	print_never_claim(std::cout, aut) << '\n';
//	auto whatever = aut->out(0);
//	spot::print_hoa(std::cout, aut) << '\n';
//
	auto init = aut->get_init_state_number();
	std::cout << init<< "        ~~INIT!!!!!!!"<< std::endl;
	const auto& dict = aut->get_dict();
	for (spot::formula ap: aut->ap())
		std::cout << ' ' << ap << " (=" << aut->get_dict()->varnum(ap) << ')';
	std::cout << '\n';
//	auto acc = aut->get_acceptance();
	std::strstream acc;

//	std::string asd;
	std::string s;
	acc << aut->get_acceptance();
	acc >> s;
	std::cout << "Acceptance: " << s << '\n';
	unsigned n = aut->num_states();
		 for (unsigned s = 0; s < n; ++s)
		     {
		       std::cout << "State " << s << ":\n";

		       // The out(s) method returns a fake container that can be
		       // iterated over as if the contents was the edges going
		       // out of s.  Each of these edge is a quadruplet
		       // (src,dst,cond,acc).  Note that because this returns
		       // a reference, the edge can also be modified.
		       for (auto& t: aut->out(s))
		         {
		           std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
		           spot::bdd_print_formula(std::cout, dict, t.cond);
		           std::cout << "\n    acc sets = " << t.acc << '\n';
		         }
		     }



	auto a = aut->acc();

	return 0;
}
