/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 */

#include <string>
#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>

#include "utility/logging/logger.hpp"
#include "common/librav_types.hpp"
#include "map/map_utils.h"
#include "map/map_config.h"
#include "map/map_info.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "path_repair/quad_path_repair.h"

using namespace librav;

void TestCase1_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_path_repair.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 32);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(2.5, 2.5);

	qplanner.ConfigGraphPlanner(map_config, 5.0, 5.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(1.8, -2.0));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

void TestCase2_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_testcase2.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(10.0, 12.5);
	//map_config.SetOriginOffset(12.5, 10.0);

	qplanner.ConfigGraphPlanner(map_config, 20.0, 25.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(11.0, -8.5));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

void TestCase3_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_testcase3.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(10.0, 12.5);
	//map_config.SetOriginOffset(12.5, 10.0);

	qplanner.ConfigGraphPlanner(map_config, 20.0, 25.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(11.0, -8.5));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

void TestCase4_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_testcase4.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 32);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(5.0, 9.0);
	//map_config.SetOriginOffset(12.5, 10.0);

	qplanner.ConfigGraphPlanner(map_config, 10.0, 18.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(8.0, 3.8));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

void TestCase5_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/map_testcase5.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(10.0, 7.5);

	qplanner.ConfigGraphPlanner(map_config, 20.0, 15.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(5.475, -7.35));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

void TestCase6_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/map_testcase6b.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(10.0, 7.5);

	qplanner.ConfigGraphPlanner(map_config, 20.0, 15.0);
	qplanner.EnablePositionAutoUpdate(true);

	//qplanner.SetGoalRefWorldPosition(Position2Dd(5.5, -6.3));
	qplanner.SetGoalRefWorldPosition(Position2Dd(1.85, -6.5));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

int main(int argc, char* argv[])
{
	// set up network first
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
	if(!lcm->good())
	{
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	// init quadrotor planner
	QuadPathRepair qplanner(lcm);

	//TestCase1_Config(qplanner);
	TestCase6_Config(qplanner);

	//LoggingHelper& logging_helper = LoggingHelper::GetInstance("quadsim_hummingbird", "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/log");

	if(qplanner.active_graph_planner_ == GraphPlannerType::NOT_SPECIFIED)
	{
		std::cout << "failed to init quad planner" << std::endl;
		return -1;
	}

	while(true)
	{
		if(qplanner.update_global_plan_)
			qplanner.UpdateGlobalPath();

		lcm->handleTimeout(0);
	}
}
