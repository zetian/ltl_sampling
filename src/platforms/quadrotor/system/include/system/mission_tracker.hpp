/* 
 * mission_tracker.hpp
 * 
 * Created on: Nov 3, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef MISSION_TRACKER_HPP
#define MISSION_TRACKER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <limits>
#include <cmath>
#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"

namespace librav {

enum class GeoMarkSource {
	NOT_SPECIFIED,
	LASER_OCTOMAP,
	PLANAR_MAP,
	VIRTUAL_POINT
};

struct GeoMark
{
	GeoMark(uint64_t id = 0):
		source(GeoMarkSource::NOT_SPECIFIED),
		source_id(0){};
	~GeoMark(){};

	Position3Dd position;
	GeoMarkSource source;
	uint64_t source_id;

	double GetHeuristic(const GeoMark& other_struct) const {
		double x_error =  this->position.x - other_struct.position.x;
		double y_error =  this->position.y - other_struct.position.y;
		double z_error =  this->position.z - other_struct.position.z;;

		return std::sqrt(std::pow(x_error,2) + std::pow(y_error,2) + std::pow(z_error, 2));
	}
};

class MissionTracker {
public:
	MissionTracker(std::shared_ptr<lcm::LCM> lcm);
	~MissionTracker() = default;

public:
	int64_t path_id_;
	bool mission_started_;
	bool replan_needed_;
	std::vector<GeoMark> active_path_;

	double remaining_path_length_;
	Position3Dd current_position_;

	void UpdateActivePathWaypoints(std::vector<GeoMark>& path);
	void UpdateCurrentPosition(Position3Dd pos);

private:
	std::shared_ptr<lcm::LCM> lcm_;

	int64_t trajectory_id_;

	double CalcRemainingPathLenght(uint32_t current_idx);
	void LcmMissionInfoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::MissionInfo_t* msg);
};

}

#endif /* MISSION_TRACKER_HPP */
