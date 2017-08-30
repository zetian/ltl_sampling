/*
 * vis_utils.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_VIS_VIS_UTILS_H_
#define PLANNING_SRC_VIS_VIS_UTILS_H_

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "map/common_types.h"

namespace srcl {

class VisUtils {
private:
	static cv::Scalar pt_color_;		// default point color
	static cv::Scalar ln_color_;		// default line color
	static cv::Scalar area_color_;		// default area color

public:
	static void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar& color = VisUtils::pt_color_);
	static void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color = VisUtils::ln_color_);
	static void FillRectangularArea(cv::Mat img, BoundingBox bbox, const cv::Scalar& color = VisUtils::area_color_);
};

}

#endif /* PLANNING_SRC_VIS_VIS_UTILS_H_ */
