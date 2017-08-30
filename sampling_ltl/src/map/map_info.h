/*
 * map_info.h
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MAP_DATA_MAP_INFO_H_
#define PLANNING_SRC_MAP_DATA_MAP_INFO_H_

namespace srcl {

// Origin is defined at the top left corner.
//	X-axis increases from left to right.
//	Y-axis increases from top to bottom.
typedef struct {
	bool vector_map;
	double world_size_x;	// in meters
	double world_size_y;
	uint32_t map_size_x;	// in pixels
	uint32_t map_size_y;

	// Offset relative to left top corner
	double origin_offset_x; // in meters
	double origin_offset_y;

	int16_t padded_top;		// in pixels
	int16_t padded_bottom;
	int16_t padded_right;
	int16_t padded_left;

	double scale_x;			// map/world
	double scale_y;

	void SetWorldSize(double x, double y)
	{
		this->world_size_x = x;
		this->world_size_y = y;

		this->scale_x = static_cast<double>(this->map_size_x)/this->world_size_x;
		this->scale_y = static_cast<double>(this->map_size_y)/this->world_size_y;
	}
} MapInfo;

}


#endif /* PLANNING_SRC_MAP_DATA_MAP_INFO_H_ */
