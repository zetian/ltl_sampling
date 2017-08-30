/*
 * hcost_tile_library.cpp
 *
 *  Created on: Feb 23, 2016
 *      Author: bscooper
 */

#include "trans_sys/hcost/hcost_tile_library.h"

namespace srcl {

HcostTileLibrary::HcostTileLibrary() {
	// TODO Auto-generated constructor stub
	N_REGION_W     = 75;
	N_REGION_PSI   = 75;
	N_REGION_SPD   = 1;
	N_REGION_TOTAL = N_REGION_W*N_REGION_PSI*N_REGION_SPD - 1;

	// CBTA numerical parameters
	N_CBTA_W_SOL = 51;
	N_CBTA_W = 101;
}

HcostTileLibrary::~HcostTileLibrary() {
	// TODO Auto-generated destructor stub
}

} /* namespace srcl */
