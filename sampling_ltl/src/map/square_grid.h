/*
 * square_grid.h
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_SQUARE_GRID_H_
#define SRC_MAP_SQUARE_GRID_H_

#include <map>
#include <vector>
#include <cstdint>

#include "graph/bds_base.h"
#include "map/common_types.h"
#include "map/task_region.h"

namespace srcl{

class SquareCell: public BDSBase<SquareCell>{
public:
	SquareCell(uint64_t id, uint32_t row, uint32_t col, BoundingBox bbox, OccupancyType occupancy):
		BDSBase<SquareCell>(id),
		occu_(occupancy)
	{
		index_.x = col;
		index_.y = row;

		bbox_ = bbox;

		location_.x = bbox_.x.min + (bbox_.x.max - bbox_.x.min)/2;
		location_.y = bbox_.y.min + (bbox_.y.max - bbox_.y.min)/2;
	}
	~SquareCell(){};

//	const uint64_t node_id_;
	Position2D index_;
	Position2D location_;
	OccupancyType occu_;
	BoundingBox bbox_;

private:
	std::vector<TaskRegion> task_region_;

public:
	std::string GetTaskRegionName()
	{
		std::string region_name;

		for(auto& it : task_region_)
			region_name += it.GetRegionName();

		return region_name;
	}

	uint32_t GetTaskRegionBitMap() const {
		uint32_t bit_map = 0;

		for(auto& it : task_region_)
			bit_map = bit_map | it.GetRegionBitMap();

		return bit_map;
	}

	void AssignRegionLabel(uint8_t label)
	{
		bool label_exist = false;

		for(auto& reg : task_region_)
		{
			if(reg.GetRegionLabel() == label)
			{
				label_exist = true;
				break;
			}
		}

		if(!label_exist) {
			TaskRegion new_region;
			new_region.SetRegionLabel(label);
			task_region_.push_back(new_region);
		}
	}

	void RemoveRegionLabel(uint8_t label)
	{
		bool label_exist = false;

		for(auto it = task_region_.begin(); it != task_region_.end(); it++)
		{
			if((*it).GetRegionLabel() == label)
			{
				label_exist = true;
				task_region_.erase(it);
				break;
			}
		}
	}

	double GetHeuristic(const SquareCell& other_struct) const{
		double x1,x2,y1,y2;

		x1 = this->location_.x;
		y1 = this->location_.y;

		x2 = other_struct.location_.x;
		y2 = other_struct.location_.y;

		// static_cast: can get wrong result to use "unsigned long" type for deduction
		long x_error = static_cast<long>(x1) - static_cast<long>(x2);
		long y_error = static_cast<long>(y1) - static_cast<long>(y2);

		double cost = std::abs(x_error) + std::abs(y_error);
		//	std::cout<< "heuristic cost: " << cost << std::endl;

		return cost;
	}
};

class SquareGrid{
public:
	SquareGrid(uint32_t row_num, uint32_t col_num, uint32_t cell_size);
	~SquareGrid();

	typedef SquareCell node_type;

public:
	std::map<uint64_t, SquareCell*> cells_;

public:
	uint32_t row_size_;
	uint32_t col_size_;
	uint32_t cell_size_;

private:
	BoundingBox CalcBoundingBox(uint64_t id);

public:
	void SetCellOccupancy(uint32_t row, uint32_t col, OccupancyType occ);
	void SetCellOccupancy(uint64_t id, OccupancyType occ);
	uint64_t GetIDFromIndex(uint32_t row, uint32_t col);
	uint64_t GetIDFromPosition(uint32_t x, uint32_t y);
	SquareCell* GetCellFromID(uint64_t id);
	std::vector<SquareCell*> GetNeighbours(uint64_t id, bool allow_diag);
};

}


#endif /* SRC_MAP_SQUARE_GRID_H_ */
