/*
 * motion_client.cpp
 *
 *  Created on: May 23, 2016
 *      Author: rdu
 */

#include <cmath>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/librav.hpp"
#include "common/librav_types.hpp"

using namespace librav;

srcl_lcm_msgs::UAVTrajectory_t GenerateTestTrajectory()
{
	srcl_lcm_msgs::UAVTrajectory_t test_traj;

	int time_stamp1 = 50;
	int time_stamp2 = time_stamp1 + 150;
	int final_time_stamp = time_stamp2 + 10;

	for(int i = 0; i < final_time_stamp; i++)
	{
		srcl_lcm_msgs::UAVTrajectoryPoint_t pt;
		pt.point_empty = false;

		double height = 0.5;
		double radius = 1.0;
		double circle_ang_vel = 180.0/180.0*3.14;

		if(i < time_stamp1)
		{
			pt.positions[0] = 0.0;
			pt.positions[1] = -1.0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.jerks[0] = 0;
			pt.jerks[1] = 0;
			pt.jerks[2] = 0;
			pt.yaw = 0;
			pt.yaw_rate = 0;
			pt.duration = 1;
		}
		else if(i < time_stamp2)
		{
			double angle = (i - time_stamp1)*0.01*circle_ang_vel;
			pt.positions[0] = radius * sin(angle);
			pt.positions[1] = -radius * cos(angle);
			pt.positions[2] = height;

			pt.velocities[0] = radius * cos(angle) * 0.01*circle_ang_vel;
			pt.velocities[1] = radius * sin(angle) * 0.01*circle_ang_vel;
			pt.velocities[2] = 0;

			pt.accelerations[0] = - radius * sin(angle) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[1] = radius * cos(angle) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[2] = 0;

			pt.jerks[0] = - radius * cos(angle) * 0.01*circle_ang_vel * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.jerks[1] = - radius * sin(angle) * 0.01*circle_ang_vel * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.jerks[2] = 0;

			pt.yaw = 0;//angle;
			pt.yaw_rate = 0;//circle_ang_vel;
			pt.duration = 1;
		}
		else
			pt.point_empty = true;

		test_traj.trajectory.push_back(pt);
	}

	test_traj.waypoint_num = test_traj.trajectory.size();

	return test_traj;
}

int main(int argc, char ** argv)
{
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    srcl_lcm_msgs::UAVTrajectory_t traj = GenerateTestTrajectory();

    std::cout << "sending request" << std::endl;

    lcm.publish("quad_controller/quad_motion_service", &traj);

    std::cout << "finished sending request" << std::endl;

    return 0;
}
