/* 
 * quad_data_broadcaster.hpp
 * 
 * Created on: May 26, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef QUAD_DATA_BROADCASTER_HPP
#define QUAD_DATA_BROADCASTER_HPP

#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include <lcm/lcm-cpp.hpp>

#include "common/librav_types.hpp"
#include "control/quad_state.hpp"

namespace librav
{

class QuadDataBroadcaster
{
  public:
	QuadDataBroadcaster() = delete;
	QuadDataBroadcaster(std::shared_ptr<lcm::LCM> lcm_ptr);

	void SendQuadStateData(const QuadState &rs);
	void SendSystemTime(uint64_t sys_t);

  private:
	std::shared_ptr<lcm::LCM> lcm_;

	void SendQuadTransform(Point3f pos, Eigen::Quaterniond quat);
	void SendLaserPoints(const std::vector<Point3f> &pts);
	void SendLaserPoints(const std::vector<Point3f> &pts, Point3f pos, Eigen::Quaterniond quat);
};

}

#endif /* QUAD_DATA_BROADCASTER_HPP */
