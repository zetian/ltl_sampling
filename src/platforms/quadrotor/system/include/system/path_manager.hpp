/* 
 * path_manager.hpp
 * 
 * Created on: Oct 31, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef PATH_MANAGER_HPP
#define PATH_MANAGER_HPP

#include <vector>
#include <memory>
#include <cstdint>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "system/quad_polyopt.hpp"

namespace librav {

class PathManager {
public:
	PathManager(std::shared_ptr<lcm::LCM> lcm);
	~PathManager() = default;

private:
	std::shared_ptr<lcm::LCM> lcm_;
	uint64_t user_path_id_;
	double CalcFlightTime(Position3Dd start, Position3Dd goal, double vel);
	std::vector<Position3Dd> GetKeyTurningWaypoints(std::vector<Position3Dd>& wps);

private:
	void LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg);
	void LcmKeyframeSetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::KeyframeSet_t* msg);

public:
	void GenerateTrajectory(KeyframeSet& kfs, uint64_t traj_id);
};

}

#endif /* PATH_MANAGER_HPP */
