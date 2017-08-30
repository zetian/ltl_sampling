/*
 * image_utils.h
 *
 *  Created on: Mar 24, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_IMAGE_UTILS_H_
#define SRC_MAP_IMAGE_UTILS_H_

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "map/common_types.h"

namespace srcl
{

typedef struct {
	int16_t top;
	int16_t bottom;
	int16_t left;
	int16_t right;
} PaddingSize;

namespace ImageUtils{

	void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
	PaddingSize PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
	PaddingSize PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);

	void CreateOccupancyMapForRRT(uint64_t width, uint64_t height, cv::OutputArray _dst);

	OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox area);
	bool IsPointNonObstacle(cv::InputArray _src, cv::Point pt);

};

}

#endif /* SRC_MAP_IMAGE_UTILS_H_ */
