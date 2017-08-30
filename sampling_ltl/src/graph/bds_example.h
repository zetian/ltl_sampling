/*
 * bds_example.h
 *
 *  Created on: Apr 15, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_BDS_EXAMPLE_H_
#define SRC_GRAPH_BDS_EXAMPLE_H_

#include "graph/bds_base.h"

namespace srcl {

/****************************************************************************/
/*				   Bundled Data Structure (BDS) Example						*/
/****************************************************************************/
/// An example BDS that can be associated with a vertex. This BDS can be
///	either a "struct" or a "class", but need to provide an implementation of
/// the GetHeuristic() function. A "data_id_" is provided with the base class.
struct BDSExample: public BDSBase<BDSExample>
{
	BDSExample(uint64_t id):
		BDSBase<BDSExample>(id){};
	~BDSExample(){};

	double GetHeuristic(const BDSExample& other_struct) const {
		return 0.0;
	}
};

}

#endif /* SRC_GRAPH_BDS_EXAMPLE_H_ */
