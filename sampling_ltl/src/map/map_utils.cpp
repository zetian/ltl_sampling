/*
 * map_utils.cpp
 *
 *  Created on: Apr 26, 2016
 *      Author: rdu
 */

#include "map/map_utils.h"

using namespace srcl;

bool MapUtils::ReadImageFromFile(std::string map_path, cv::OutputArray _dst)
{
	cv::Mat image;

	if(!map_path.empty()) {
		// read map image
		image = imread(map_path, cv::IMREAD_GRAYSCALE);

		if (!image.data) {
			printf("No image data \n");
			return false;
		}
		else
		{
			image.copyTo(_dst);
			return true;
		}
	}
	else
		return false;
}

std::shared_ptr<SquareGrid> MapUtils::CreateSquareGrid(uint32_t row_size, uint32_t col_size, uint32_t cell_size)
{
	return std::make_shared<SquareGrid>(row_size,col_size,cell_size);
}

Position2Dd MapUtils::CoordinatesFromMapToWorld(Position2D map_pos, MapInfo info)
{
	Position2Dd rpos;

	rpos.x = map_pos.x / info.scale_x;
	rpos.y = map_pos.y / info.scale_y;

	if(rpos.x > info.world_size_x)
		rpos.x = info.world_size_x;
	if(rpos.y > info.world_size_y)
		rpos.y = info.world_size_y;

//	std::cout << std::endl;
//	std::cout << "map scale: " << info.scale_x << " , " << info.scale_y << std::endl;
//	std::cout << "input: " << map_pos.x << " , " << map_pos.y << std::endl;
//	std::cout << "convertion result: " << rpos.x << " , " << rpos.y << std::endl;

	return rpos;
}

Position2D MapUtils::CoordinatesFromWorldToMap(Position2Dd world_pos, MapInfo info)
{
	Position2D rpos;

	rpos.x = world_pos.x * info.scale_x;
	rpos.y = world_pos.y * info.scale_y;

	if(rpos.x > info.map_size_x)
		rpos.x = info.map_size_x;
	if(rpos.y > info.map_size_y)
		rpos.y = info.map_size_y;

//	std::cout << std::endl;
//	std::cout << "input: " << world_pos.x << " , " << world_pos.y << std::endl;
//	std::cout << "output: " << rpos.x << " , " << rpos.y << std::endl;

	return rpos;
}

Position2D MapUtils::CoordinatesFromPaddedToOriginal(Position2D pad_pos, MapInfo info)
{
	Position2D rpos;

	if(pad_pos.x > info.map_size_x + info.padded_left)
		rpos.x = info.map_size_x;
	else if(pad_pos.x <= info.padded_left)
		rpos.x = 0;
	else
		rpos.x = pad_pos.x - info.padded_left;

	if(pad_pos.y > info.map_size_y + info.padded_top)
		rpos.y = info.map_size_y;
	else if(pad_pos.y <= info.padded_top)
		rpos.y = 0;
	else
		rpos.y = pad_pos.y - info.padded_top;

//	std::cout << std::endl;
//	std::cout << "input: " << pad_pos.x << " , " << pad_pos.y << std::endl;
//	std::cout << "output: " << rpos.x << " , " << rpos.y << std::endl;

	return rpos;
}

Position2D MapUtils::CoordinatesFromOriginalToPadded(Position2D ori_pos, MapInfo info)
{
	Position2D rpos;

	rpos.x = ori_pos.x + info.padded_left;
	rpos.y = ori_pos.y + info.padded_top;

//	std::cout << std::endl;
//	std::cout << "input: " << ori_pos.x << " , " << ori_pos.y << std::endl;
//	std::cout << "output: " << rpos.x << " , " << rpos.y << std::endl;

	return rpos;
}
