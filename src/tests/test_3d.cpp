#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <bitset>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <random>


#include "trajectory/dubins_steer.h"
#include "trans_sys/spot_hoa_interpreter.h"

#include "sampling/ltl_sampling_3d.h"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/acel_lcm_msgs.hpp"

#include "common/librav_types.hpp"

#include "common/quad_flattraj.h"
#include "polyopt/quad_polyopt.h"
#include "polyopt/gurobi_solver/gurobi_polyopt.h"

using namespace acel;

double GetRefactoredTime(double ts, double te, double t)
{
	if(t < ts)
		t = ts;
	if(t > te)
		t = te;

	return (t - ts) / (te - ts);
}


int main()
{
    srand(time(NULL));
    lcm::LCM lcm;
    double EPSILON = 6;
    double RADIUS = 12;
    std::string ltl_formula = "(<> p0) && (<> p1) && (<> p2)";
    std::vector<std::string> buchi_regions;
    buchi_regions.push_back("p0");
    buchi_regions.push_back("p1");
    buchi_regions.push_back("p2");
    std::vector<int> indep_set = {0, 1, 2};
    double work_space_size_x = 100;
    double work_space_size_y = 100;
    double work_space_size_z = 100;
    std::vector<double> init_state = {10, 10, 10};

    LTL_Sampling3d ltl_sampling_simple;
    ltl_sampling_simple.read_formula(ltl_formula, buchi_regions, indep_set);
    ltl_sampling_simple.init_workspace(work_space_size_x, work_space_size_y, work_space_size_z);
    // Publish workspace size for visualization
    sampling_3d::workspace_size_data_3d space_data;
    space_data.size_x = work_space_size_x;
    space_data.size_y = work_space_size_y;
    space_data.size_z = work_space_size_z;
    lcm.publish("WORKSPACE", &space_data);

    // sampling::all_regions all_regions_;
    // all_regions_.num_region = 3;

    std::pair <double, double> position_x (20, 35);
    std::pair <double, double> position_y (30, 45);
    std::pair <double, double> position_z (10, 30);
    ltl_sampling_simple.set_interest_region(position_x, position_y, position_z, 0);
    sampling_3d::region_data_3d r_data;
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    r_data.position_z[0] =  position_z.first;
    r_data.position_z[1] =  position_z.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[0] = r_data;

    position_x = std::make_pair(55, 95);
    position_y = std::make_pair(55, 95);
    position_z = std::make_pair(55, 95);
    ltl_sampling_simple.set_interest_region(position_x, position_y, position_z, 1);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    r_data.position_z[0] =  position_z.first;
    r_data.position_z[1] =  position_z.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[1] = r_data;

    position_x = std::make_pair(10, 20);
    position_y = std::make_pair(80, 90);
    position_y = std::make_pair(80, 90);
    ltl_sampling_simple.set_interest_region(position_x, position_y, position_z, 2);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    r_data.position_z[0] =  position_z.first;
    r_data.position_z[1] =  position_z.second;
    lcm.publish("REGION", &r_data);
    // all_regions_.regions[2] = r_data;
    position_x = std::make_pair(35, 62);
    position_y = std::make_pair(35, 40);
    position_z = std::make_pair(20, 80);
    ltl_sampling_simple.set_obstacle(position_x, position_y, position_z);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    r_data.position_z[0] =  position_z.first;
    r_data.position_z[1] =  position_z.second;
    lcm.publish("OBSTACLE", &r_data);

    position_x = std::make_pair(15, 40);
    position_y = std::make_pair(65, 70);
    position_z = std::make_pair(20, 70);
    ltl_sampling_simple.set_obstacle(position_x, position_y, position_z);
    r_data.position_x[0] =  position_x.first;
    r_data.position_x[1] =  position_x.second;
    r_data.position_y[0] =  position_y.first;
    r_data.position_y[1] =  position_y.second;
    r_data.position_z[0] =  position_z.first;
    r_data.position_z[1] =  position_z.second;
    lcm.publish("OBSTACLE", &r_data);

    ltl_sampling_simple.set_init_state(init_state);
    int interation = 1000;

    ltl_sampling_simple.start_sampling(interation);
    std::cout << "~~~~~~done~~~~~~" << std::endl;
    std::vector<std::vector<double>> path = ltl_sampling_simple.get_path();

        std::vector<std::vector<double>> traj(3);


    librav::KeyframeSet kfs;
    for (int i = 0; i < path.size(); i++) {
        sampling_3d::sample_data_3d node_data;
        node_data.state[0] = path[i][0];
        node_data.state[1] = path[i][1];
        node_data.state[2] = path[i][2];
        lcm.publish("SAMPLE", &node_data);


        librav::Keyframe kf;
        kf.position[0] = path[i][0];
        // kf.velocity[0] = 1;
        kf.position[1] = path[i][1];
        // kf.velocity[1] = 1;
        // kf.velocity[2] = 0;
        kf.position[2] = path[i][2];
        kf.vel_constr = false;
        kf.yaw = 0;
        kfs.keyframes.push_back(kf);
    }
    srcl_lcm_msgs::PolynomialCurve_t poly_msg;
	uint8_t kf_num = kfs.keyframes.size();
    poly_msg.wp_num = kfs.keyframes.size();
    librav::QuadPolyOpt traj_opt_;
    librav::QuadFlatTraj flat_traj_;
    traj_opt_.InitOptWithCorridorJointMatrices(kf_num, 20, 4);
    traj_opt_.SetYawPolynomialOrder(3);
    // traj_opt_.InitOptJointMatrices(kf_num);
    for(int i = 0; i < kfs.keyframes.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0,i) = kfs.keyframes[i].position[0];
		traj_opt_.keyframe_y_vals_(0,i) = kfs.keyframes[i].position[1];
		traj_opt_.keyframe_z_vals_(0,i) = kfs.keyframes[i].position[2];

		if(kfs.keyframes[i].vel_constr)
		{
			traj_opt_.keyframe_x_vals_(1,i) = kfs.keyframes[i].velocity[0];
			traj_opt_.keyframe_y_vals_(1,i) = kfs.keyframes[i].velocity[1];
			traj_opt_.keyframe_z_vals_(1,i) = kfs.keyframes[i].velocity[2];
		}
		else
		{
			traj_opt_.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();
		}

		traj_opt_.keyframe_x_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(2,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(3,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_yaw_vals_(0,i) = kfs.keyframes[i].yaw;
		traj_opt_.keyframe_yaw_vals_(1,i) = std::numeric_limits<float>::infinity();

		//traj_opt_.keyframe_ts_(0,i) = i * 1.0;

		if(i == 0) {
			traj_opt_.keyframe_ts_(0,i) = 0;
		}
		else {
			traj_opt_.keyframe_ts_(0,i) = traj_opt_.keyframe_ts_(0,i-1) + 1.0;

//			Position3Dd start(traj_opt_.keyframe_x_vals_(0,i-1), traj_opt_.keyframe_y_vals_(0,i-1), traj_opt_.keyframe_z_vals_(0,i-1));
//			Position3Dd goal(traj_opt_.keyframe_x_vals_(0,i), traj_opt_.keyframe_y_vals_(0,i), traj_opt_.keyframe_z_vals_(0,i));
//
//			double tp = CalcFlightTime(start, goal, 2.0);
//			traj_opt_.keyframe_ts_(0,i) = traj_opt_.keyframe_ts_(0,i-1) + tp*5;
        }
		srcl_lcm_msgs::WayPoint_t wpoint;
		wpoint.positions[0] = kfs.keyframes[i].position[0];
		wpoint.positions[1] = kfs.keyframes[i].position[1];
		wpoint.positions[2] = kfs.keyframes[i].position[2];
        poly_msg.waypoints.push_back(wpoint);
        
        traj[0].push_back(kfs.keyframes[i].position[0]);
        traj[1].push_back(kfs.keyframes[i].position[1]);
        traj[2].push_back(kfs.keyframes[i].position[2]);
    }
    traj_opt_.keyframe_x_vals_(1,0) = 0.0;
	traj_opt_.keyframe_y_vals_(1,0) = 0.0;
	traj_opt_.keyframe_z_vals_(1,0) = 0.0;

	traj_opt_.keyframe_x_vals_(2,0) = 0.0;
	traj_opt_.keyframe_y_vals_(2,0) = 0.0;
	traj_opt_.keyframe_z_vals_(2,0) = 0.0;

	traj_opt_.keyframe_x_vals_(3,0) = 0.0;
	traj_opt_.keyframe_y_vals_(3,0) = 0.0;
	traj_opt_.keyframe_z_vals_(3,0) = 0.0;

	traj_opt_.keyframe_yaw_vals_(1,0) = 0;

	traj_opt_.keyframe_x_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(2,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_y_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_z_vals_(3,kf_num - 1) = 0;

	traj_opt_.keyframe_yaw_vals_(1,kf_num - 1) = 0;

    //traj_opt_.OptimizeFlatTrajJoint();
    
    
    bool result = traj_opt_.OptimizeFlatTrajWithCorridorJoint();
    if(result)
	{
		poly_msg.seg_num = traj_opt_.flat_traj_.traj_segs_.size();
		for(auto& seg : traj_opt_.flat_traj_.traj_segs_)
		{
			srcl_lcm_msgs::PolyCurveSegment_t seg_msg;

			seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
			seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
			seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
			seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
			for(auto& coeff:seg.seg_x.param_.coeffs){
                seg_msg.coeffs_x.push_back(coeff);
                // traj[0].push_back(coeff);
            }
				
			for(auto& coeff:seg.seg_y.param_.coeffs){
                seg_msg.coeffs_y.push_back(coeff);
                // traj[1].push_back(coeff);
            }
				
			for(auto& coeff:seg.seg_z.param_.coeffs)
				seg_msg.coeffs_z.push_back(coeff);
			for(auto& coeff:seg.seg_yaw.param_.coeffs)
				seg_msg.coeffs_yaw.push_back(coeff);

			seg_msg.t_start = seg.t_start;
			seg_msg.t_end = seg.t_end;

			poly_msg.segments.push_back(seg_msg);
		}
		poly_msg.start_time.time_stamp = kfs.start_time;
		poly_msg.trajectory_id = 0;

		poly_msg.scaling_factor = 0.3;

		// lcm.publish("quad_planner/trajectory_polynomial", &poly_msg);
    }

    std::vector<librav::Position3Dd> waypoints_;
    flat_traj_.clear();
    for(auto& seg:poly_msg.segments)
	{
		std::vector<std::vector<double>> segcoeffs;

		segcoeffs.push_back(seg.coeffs_x);
		segcoeffs.push_back(seg.coeffs_y);
		segcoeffs.push_back(seg.coeffs_z);
		segcoeffs.push_back(seg.coeffs_yaw);

		//std::cout << "start time: " << seg.t_start << " , end time: " << seg.t_end << std::endl;

		flat_traj_.AddTrajSeg(segcoeffs, seg.t_start, seg.t_end);
	}
    for(auto& wp:poly_msg.waypoints)
		waypoints_.push_back(librav::Position3Dd(wp.positions[0], wp.positions[1], wp.positions[2]));


    std::cout << "num of segments: " << poly_msg.seg_num << std::endl;

    // for (int i = 0; i < poly_msg.seg_num; i++) {
    //     std::cout << "segment " << i << std::endl;
    //     std::cout << "start time: " << poly_msg.segments[i].t_start << std::endl;
    //     std::cout << "end time: " << poly_msg.segments[i].t_end << std::endl;
    // }
    double f_time = poly_msg.segments[poly_msg.seg_num - 1].t_end;
    std::cout << "final end time: " << f_time << std::endl;
    double time_step = 0.05;
    double cur_time = 0;

    std::vector<librav::UAVTrajectoryPoint> all_traj;
    // Build traj
    while (cur_time < f_time) {
        librav::UAVTrajectoryPoint pt;
        int seg_idx = 0;
		for(seg_idx = 0; seg_idx < flat_traj_.traj_segs_.size(); seg_idx++)
		{
			if(cur_time >= flat_traj_.traj_segs_[seg_idx].t_start && cur_time <= flat_traj_.traj_segs_[seg_idx].t_end){
				break;
            }

        }   

        double seg_t_start = flat_traj_.traj_segs_[seg_idx].t_start;
		double seg_t_end = flat_traj_.traj_segs_[seg_idx].t_end;
        double t_factor = GetRefactoredTime(seg_t_start, seg_t_end, cur_time);
        pt.positions[0] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 0, t_factor);
		pt.positions[1] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 0, t_factor);
		pt.positions[2] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 0, t_factor);

		pt.velocities[0] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 1, t_factor);
		pt.velocities[1] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 1, t_factor);
		pt.velocities[2] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 1, t_factor);

		pt.accelerations[0] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 2, t_factor);
		pt.accelerations[1] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 2, t_factor);
		pt.accelerations[2] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 2, t_factor);

		pt.jerks[0] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 3, t_factor);
		pt.jerks[1] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 3, t_factor);
		pt.jerks[2] = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 3, t_factor);
        // int32_t fpt_idx = FindFurthestPointWithinRadius(waypoints_,seg_idx, 5.0);
        // Eigen::Vector3d furthest_pt_vec(waypoints_[fpt_idx].x, waypoints_[fpt_idx].y, 0);
        pt.yaw = librav::PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 0, t_factor);
        pt.yaw_rate = 0;
        double dist = 0;
        if(seg_idx == flat_traj_.traj_segs_.size() - 1)
		{
			dist = std::sqrt(std::pow(pt.positions[0] - waypoints_.back().x,2) +
					std::pow(pt.positions[1] - waypoints_.back().y,2) + std::pow(pt.positions[2] - waypoints_.back().z,2));
		}
		else
		{
			// calc remaining distance of the current segment
			//dist += std::sqrt(std::pow(pt.positions[0] - waypoints_[seg_idx + 1].x,2) +
			//		std::pow(pt.positions[1] - waypoints_[seg_idx + 1].y,2) + std::pow(pt.positions[2] - waypoints_[seg_idx + 1].z,2));

			// calc other waypoints
			//for(int i = seg_idx + 1; i < flat_traj_.traj_segs_.size() - 1; i++)
			for(int i = seg_idx; i < flat_traj_.traj_segs_.size() - 1; i++)
			{
				dist += std::sqrt(std::pow(waypoints_[i].x - waypoints_[i + 1].x,2) +
									std::pow(waypoints_[i].y - waypoints_[i + 1].y,2)
									+ std::pow(waypoints_[i].z - waypoints_[i + 1].z,2));
			}
        }
        if(dist < 0.01)
			dist = 0;
        // remaining_dist_ = dist;

        cur_time += time_step;
        all_traj.push_back(pt);
        // Vis for debug
        // sampling::sample_data node_data;
        // node_data.state[0] = pt.positions[0];
        // node_data.state[1] = pt.positions[1];
        // lcm.publish("SAMPLE", &node_data);


    }

    sampling_3d::path_data_3d path_data_;
    path_data_.num_state = all_traj.size();
    path_data_.state_x.resize(path_data_.num_state);
    path_data_.state_y.resize(path_data_.num_state);
    path_data_.state_z.resize(path_data_.num_state);
    for (int i = 0; i < path_data_.num_state; i++) {
        path_data_.state_x[i] = all_traj[i].positions[0];
        
        path_data_.state_y[i] = all_traj[i].positions[1];
        path_data_.state_y[i] = all_traj[i].positions[2];
    }
    std::cout << "Length of the solution path: " << path_data_.num_state << std::endl;
    lcm.publish("PATH", &path_data_);






    // sampling::path_data path_data_;
    // path_data_.num_state = traj[0].size();
    // path_data_.state_x.resize(path_data_.num_state);
    // path_data_.state_y.resize(path_data_.num_state);
    // for (int i = 0; i < path_data_.num_state; i++) {
    //     path_data_.state_x[i] = traj[0][i];
        
    //     path_data_.state_y[i] = traj[1][i];
    // }
    // std::cout << "Length of the solution path: " << path_data_.num_state << std::endl;
    // lcm.publish("PATH", &path_data_);
    
    sampling_3d::sample_draw_3d draw;
    draw.if_draw = true;
    // lcm.publish("DRAW_REGION", &draw);
    lcm.publish("DRAW_SAMPLE", &draw);




    // sampling::path_data_3d path_data_;
    // path_data_.num_state = path.size();
    // path_data_.state_x.resize(path_data_.num_state);
    // path_data_.state_y.resize(path_data_.num_state);
    // path_data_.state_z.resize(path_data_.num_state);
    // for (int i = 0; i < path.size(); i++) {
        
    //     path_data_.state_x[i] = path[i][0];
        
    //     path_data_.state_y[i] = path[i][1];
    //     path_data_.state_z[i] = path[i][2];
    // }
    // std::cout << "Length of the solution path: " << path.size() << std::endl;
    // lcm.publish("PATH", &path_data_);
    // sampling::sample_draw draw;
    // draw.if_draw = true;
    // // lcm.publish("DRAW_REGION", &draw);
    // lcm.publish("DRAW_SAMPLE", &draw);



    
    return 0;
}