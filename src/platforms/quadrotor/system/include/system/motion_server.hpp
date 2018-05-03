/* 
 * motion_server.hpp
 * 
 * Created on: May 23, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef MOTION_SERVER_HPP
#define MOTION_SERVER_HPP

#include <vector>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "system/quad_flattraj_handler.hpp"

namespace librav {

enum class MotionMode {
	POLYNOMIAL,
	WAYPOINTS,
	USER_CMDS,
	POS_STEP_RESPONSE
};

class MotionServer {
public:
	MotionServer() = delete;
	MotionServer(std::shared_ptr<lcm::LCM> lcm);
	~MotionServer() = default;

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::unique_ptr<QuadFlatTrajHandler> polytraj_handler_;

private:
	bool goal_completed_;
	UAVTrajectory active_goal_;
	uint64_t ms_count_;
	uint64_t waypoint_idx_;
	time_stamp current_sys_time_;

	MotionMode mode_;

private:
	UAVTrajectory GenerateTestTrajectory();

public:
	void SetMotionMode(MotionMode mode) { mode_ = mode; };
	void LcmSysTimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::TimeStamp_t* msg);
	UAVTrajectoryPoint GetCurrentDesiredState(time_stamp t);

	void LcmUserGoalHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::UAVTrajectory_t* msg);
	void SetMotionGoal(UAVTrajectory& goal);
	void AbortActiveMotion();
	double ReportActiveMotionProgress();
	void SetGoalCompleted();
	UAVTrajectoryPoint GetCurrentUserDefinedPose();

	void ReportProgress(void);
};

}

#endif /* MOTION_SERVER_HPP */
