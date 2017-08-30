/*
 * map_type.h
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MAP_MAP_TYPE_H_
#define PLANNING_SRC_MAP_MAP_TYPE_H_

#include <memory>

#include "opencv2/opencv.hpp"

#include "map/map_info.h"

namespace srcl {

template<typename DataModelType>
struct Map{
	MapInfo info;

	std::shared_ptr<DataModelType> data_model;
	cv::Mat input_image;
	cv::Mat padded_image;
};

template<typename T>
using Map_t = struct Map<T>;

}

#endif /* PLANNING_SRC_MAP_MAP_TYPE_H_ */
