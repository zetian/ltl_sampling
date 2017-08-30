/*
 * square_grid.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

//#include <iostream>
#include "map/square_grid.h"

using namespace srcl;

SquareGrid::SquareGrid(uint32_t row_num, uint32_t col_num, uint32_t cell_size):
		row_size_(row_num), col_size_(col_num), cell_size_(cell_size)
{
	for(uint32_t i = 0; i < row_num; i++)
		for(uint32_t j = 0; j < col_num; j++)
		{
			uint64_t new_id = i * col_num + j;
			SquareCell* new_cell = new SquareCell(new_id, i, j, CalcBoundingBox(new_id), OccupancyType::FREE);
			cells_[new_id] = new_cell;
		}
}

SquareGrid::~SquareGrid(){
	std::map<uint64_t, SquareCell*>::iterator itm;
	for(itm = cells_.begin(); itm != cells_.end(); itm++)
		delete itm->second;

	cells_.clear();
}

void SquareGrid::SetCellOccupancy(uint32_t row, uint32_t col, OccupancyType occ)
{
	SetCellOccupancy(row+col*col_size_, occ);
}
void SquareGrid::SetCellOccupancy(uint64_t id, OccupancyType occ)
{
	cells_[id]->occu_ = occ;
}

uint64_t SquareGrid::GetIDFromIndex(uint32_t row, uint32_t col)
{
	return row * col_size_ + col;
}

uint64_t SquareGrid::GetIDFromPosition(uint32_t x, uint32_t y)
{
	uint32_t row, col;

	col = x / cell_size_;
	row = y / cell_size_;

	return GetIDFromIndex(row, col);
}

SquareCell* SquareGrid::GetCellFromID(uint64_t id)
{
	auto it = cells_.find(id);

	if(it != cells_.end())
		return (*it).second;
	else
		return nullptr;
}

BoundingBox SquareGrid::CalcBoundingBox(uint64_t id)
{
	BoundingBox bbox;
	uint32_t x,y;
	x = id%col_size_;
	y = id/col_size_;
	bbox.x.min = x*cell_size_;
	bbox.x.max = bbox.x.min + cell_size_ - 1;
	bbox.y.min = y*cell_size_;
	bbox.y.max = bbox.y.min + cell_size_ - 1;

	return bbox;
}

std::vector<SquareCell*> SquareGrid::GetNeighbours(uint64_t id, bool allow_diag)
{
	std::vector<SquareCell*> neighbours;

	uint32_t x,y;
	x = cells_[id]->index_.x;
	y = cells_[id]->index_.y;

	// not consider diagonal cells
	if(allow_diag)
	{
		Position2D pos[8];

		pos[0].x = x - 1;
		pos[0].y = y - 1;

		pos[1].x = x;
		pos[1].y = y - 1;

		pos[2].x = x + 1;
		pos[2].y = y - 1;

		pos[3].x = x - 1;
		pos[3].y = y;

		pos[4].x = x + 1;
		pos[4].y = y;

		pos[5].x = x - 1;
		pos[5].y = y + 1;

		pos[6].x = x;
		pos[6].y = y + 1;

		pos[7].x = x + 1;
		pos[7].y = y + 1;

		for(int i = 0; i < 8; i++)
		{
			if(pos[i].x < col_size_ && pos[i].y < row_size_)
				neighbours.push_back(cells_[pos[i].y * col_size_ + pos[i].x]);
		}
	}
	else
	{
		Position2D pos[4];

		pos[0].x = x;
		pos[0].y = y + 1;

		pos[1].x = x;
		pos[1].y = y - 1;

		pos[2].x = x + 1;
		pos[2].y = y;

		pos[3].x = x - 1;
		pos[3].y = y;

		for(int i = 0; i < 4; i++)
		{
			if(pos[i].x < col_size_ && pos[i].y < row_size_)
				neighbours.push_back(cells_[pos[i].y * col_size_ + pos[i].x]);
		}
	}

	return neighbours;
}

