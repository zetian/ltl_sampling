/*
 * sgrid_builder.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#include <map/image_utils.h>
#include <map/sgrid_builder.h>

using namespace srcl;
using namespace cv;

std::shared_ptr<SquareGrid> SGridBuilder::BuildSquareGrid(cv::InputArray _src, uint32_t cell_size)
{
	Mat image_bin;
	Mat image_map;
	Mat src = _src.getMat();

	// binarize grayscale image
	ImageUtils::BinarizeImage(src, image_bin, 200);

	// pad image to 2^n on each side so that we can calculate
	//	the dimension of the grid more conveniently
	ImageUtils::PadImageTo2Exp(image_bin, image_map);

	// create square grid
	uint32_t row_num = image_map.rows / cell_size;
	uint32_t col_num = image_map.cols / cell_size;
	std::shared_ptr<SquareGrid> sgrid = std::make_shared<SquareGrid>(row_num,col_num, cell_size);

	// set occupancy of each cell
	for(uint32_t i = 0; i < row_num; i++)
		for(uint32_t j = 0; j < col_num; j++)
		{
			uint32_t id = sgrid->GetIDFromIndex(i,j);
			if(ImageUtils::CheckAreaOccupancy(image_map, sgrid->cells_[id]->bbox_) == OccupancyType::OCCUPIED ||
					ImageUtils::CheckAreaOccupancy(image_map, sgrid->cells_[id]->bbox_) == OccupancyType::MIXED)
				sgrid->SetCellOccupancy(id, OccupancyType::OCCUPIED);
		}

	return sgrid;
}

Map_t<SquareGrid> SGridBuilder::BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size)
{
	Mat src = _src.getMat();

	Map_t<SquareGrid> map;
	map.input_image = src;

	// binarize grayscale image
	Mat image_bin;
	ImageUtils::BinarizeImage(src, image_bin, 200);

	// pad image to 2^n on each side so that we can calculate
	//	the dimension of the grid more conveniently
	PaddingSize psize = ImageUtils::PadImageTo2Exp(image_bin, map.padded_image);

	// generate map info
	map.info.vector_map = false;
	map.info.map_size_x = map.input_image.cols - 1;
	map.info.map_size_y = map.input_image.rows - 1;
	map.info.padded_top = psize.top;
	map.info.padded_bottom = psize.bottom;
	map.info.padded_right = psize.right;
	map.info.padded_left = psize.left;

	map.info.scale_x = 1; //static_cast<double>(map.info.map_size_x)/map.info.world_size_x;
	map.info.scale_y = 1; //static_cast<double>(map.info.map_size_y)/map.info.world_size_y;

	// create square grid
	map.data_model = SGridBuilder::BuildSquareGrid(_src, cell_size);

	return map;
}
