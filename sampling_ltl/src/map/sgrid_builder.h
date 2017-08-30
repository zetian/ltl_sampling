/*
 * sgrid_builder.h
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_SGRID_BUILDER_H_
#define SRC_MAP_SGRID_BUILDER_H_

#include <vector>
#include <tuple>
#include <cstdint>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "map/square_grid.h"
#include "map/map_type.h"

namespace srcl {

namespace SGridBuilder
{
	std::shared_ptr<SquareGrid> BuildSquareGrid(cv::InputArray _src, uint32_t cell_size);
	Map_t<SquareGrid> BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size);
};

}

#endif /* SRC_MAP_SGRID_BUILDER_H_ */
