/*
 * dubins_steer.cpp
 *
 *  Created on: 25 Nov 2016
 *      Author: Zetian
 */
#include "trajectory/dubins_steer.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>


 typedef struct
 {
 	double a;           
 	double b;
 	double s;          
 	double g;         
 	double length;               
 } OutputData;

 double fmodr( double x, double y)
 {
 	return x - y*floor(x/y);
 }

 double mod2pi( double theta )
 {
 	return fmodr( theta, 2 * M_PI );
 }

 std::vector<double> LinSpace(double a, double b, int n) {
	std::vector<double> array;
	if (n==1){
		array.push_back(a);
		array.push_back(b);
		return array;
	}

	if (std::abs(a - b) < 1e-3) {	
		for (int i = 0; i < n; i++){
			array.push_back(a);
		}
		return array;
	}
   if (a < b) {
	   double step = (b-a) / (n-1);
	   while(a < b) {
		   array.push_back(a);
		   a += step;
	  }
	  array.push_back(b);
	  
   }
   else {
		double step = (a-b) / (n-1);
		while(b < a) {
			array.push_back(a);
			a -= step;
		}
	array.push_back(b);
	}

   return array;
}

std::vector<double> GetConfigurationF(std::vector<double> z_0, std::vector<double> z_f){
	double x = z_f[0] - z_0[0];
	double y = z_f[1] - z_0[1];
	double yaw_0 = z_0[2];
	std::vector<double> configuration_f({x*cos(-yaw_0)-y*sin(-yaw_0),x*sin(-yaw_0)+y*cos(-yaw_0),mod2pi(z_f[2] - z_0[2])});
	return configuration_f;
}

OutputData DubinLRL(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double A = configuration_f[0] - radius_L*sin(configuration_f[2]);
	double B = configuration_f[1] - radius_L + radius_L*cos(configuration_f[2]);
	double C2 = pow(A,2) + pow(B,2);
	double tmp = 1 - C2/2/pow(radius_L + radius_L,2);
	double a, b, g;
	if (std::abs(tmp) <=1){
		b = 2*M_PI - acos(tmp);
		double Ay = (radius_L+radius_R) / C2 * (A * (1-cos(b)) + B * sin(b));
		double Ax = (radius_L+radius_R) / C2 * (B * (1-cos(b)) - A * sin(b));
		a = mod2pi(atan2(Ay, Ax));
		g = mod2pi(configuration_f[2] - a + b);
	}
	else{
		a = 100;
		b = 100;
		g = 100;
	}
	double length = radius_L * ( a + g ) + radius_R * b;
	output_data.a = a;
	output_data.b = b;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

OutputData DubinRLR(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double A = configuration_f[0] - radius_R*sin(configuration_f[2]);
	double B = configuration_f[1] - radius_R + radius_R*cos(configuration_f[2]);
	double C2 = pow(A,2) + pow(B,2);
	double tmp = 1 - C2/2/pow(radius_L + radius_L,2);
	double a, b, g;
	if (std::abs(tmp) <=1){
		b = 2*M_PI - acos(tmp);
		double Ay = (radius_L+radius_R) / C2 * (A * (1-cos(b)) - B * sin(b));
		double Ax = (radius_L+radius_R) / C2 * (B * (1-cos(b)) + A * sin(b));
		a = mod2pi(atan2(Ay, Ax ));
		g = mod2pi(-configuration_f[2] - a + b);
	}
	else{
		a = 100;
		b = 100;
		g = 100;
	}
	double length = radius_R * ( a + g ) + radius_L * b;
	output_data.a = a;
	output_data.b = b;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

OutputData DubinLSL(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double a, s, g;
	double A = configuration_f[0] - radius_L * sin(configuration_f[2]);
	double B = configuration_f[1] + radius_L * cos(configuration_f[2]) - radius_L;
	double srat = pow(A,2) + pow(B,2);
	if (srat>=0){
		s = sqrt(srat);
		a = mod2pi(atan2(B, A));
		g = mod2pi(configuration_f[2] - a);
	}
	else{
		a = 100;
		s = 100;
		g = 100;
	}
	double length = radius_L * (a + g) + s; 
	output_data.a = a;
	output_data.s = s;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

OutputData DubinLSR(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double a, s, g;
	double A = configuration_f[0] + radius_R * sin(configuration_f[2]);
	double B = -configuration_f[1] + radius_L + radius_R * cos(configuration_f[2]);
	double C2 = pow(A,2) + pow(B,2);
	double srat = C2 - pow(radius_R+radius_L,2);

	if (srat>=0){
		s = sqrt(srat);
		double aa = atan2( A/C2*(radius_R+radius_L) - B/C2*s, B/C2*(radius_R+radius_L) + A/C2*s);
		a = mod2pi(aa);
		g = mod2pi(-configuration_f[2] + a);
	}
	else{
		a = 100;
		s = 100;
		g = 100;
	}
	double length = s + radius_L * a + radius_R * g;
	output_data.a = a;
	output_data.s = s;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

OutputData DubinRSL(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double a, s, g;
	double A = configuration_f[0] - radius_L * sin(configuration_f[2]);
	double B = configuration_f[1] + radius_R + radius_L * cos(configuration_f[2]);
	double C2 = pow(A,2) + pow(B,2);
	double srat = C2 - pow(radius_R+radius_L,2);

	if (srat>=0){
		s = sqrt(srat);
		double aa = atan2( A/C2*(radius_R+radius_L) - B/C2*s, B/C2*(radius_R+radius_L) + A/C2*s);
		a = mod2pi(aa);
		g = mod2pi(configuration_f[2] + a);
	}
	else{
		a = 100;
		s = 100;
		g = 100;
	}
	double length = s + radius_L * a + radius_R * g;
	output_data.a = a;
	output_data.s = s;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

OutputData DubinRSR(std::vector<double> configuration_f, double radius_L, double radius_R){
	OutputData output_data;
	double a, s, g;
	double A = configuration_f[0] + radius_R * sin(configuration_f[2]);
	double B = -configuration_f[1] - radius_R + radius_R * cos(configuration_f[2]);
	double srat = pow(A,2) + pow(B,2);

	if (srat>=0){
		s = sqrt(srat);
		a = mod2pi(atan2(B, A));
		g = mod2pi(2 * M_PI - configuration_f[2] - a);
	}
	else{
		a = 100;
		s = 100;
		g = 100;
	}
	double length = radius_R * (a + g) + s ;
	output_data.a = a;
	output_data.s = s;
	output_data.g = g;
	output_data.length = length;
	return output_data;
}

std::vector<OutputData> asymDubins(std::vector<double> configuration_f, double radius_L, double radius_R){
	std::vector<OutputData> all_output_data;
	all_output_data.push_back(DubinLSL(configuration_f, radius_L, radius_R));
	all_output_data.push_back(DubinRSR(configuration_f, radius_L, radius_R));
	all_output_data.push_back(DubinLSR(configuration_f, radius_L, radius_R));
	all_output_data.push_back(DubinRSL(configuration_f, radius_L, radius_R));
	all_output_data.push_back(DubinLRL(configuration_f, radius_L, radius_R));
	all_output_data.push_back(DubinRLR(configuration_f, radius_L, radius_R));
	return all_output_data;
}

double DubinsSteer::GetDubinsCurveLength(std::vector<double> z_0, std::vector<double> z_f, double radius_L, double radius_R){
	double min_length;
	std::vector<double> configuration_f;
	configuration_f = GetConfigurationF(z_0, z_f);
	std::vector<OutputData> all_output_data;
	all_output_data = asymDubins(configuration_f, radius_L, radius_R);
	std::vector<double> all_length;
	for (auto it = all_output_data.begin();it != all_output_data.end(); it++){
		all_length.push_back((*it).length);
	}
	min_length = *std::min_element(all_length.begin(), all_length.end());
	return min_length;
}



DubinsSteer::SteerData DubinsSteer::GetDubinsTrajectoryPointWise(std::vector<double> z_0, std::vector<double> z_f, double radius_L, double radius_R){
	double x_0 = z_0[0];	double y_0 = z_0[1];
	double x_f = z_f[0];	double y_f = z_f[1];
	double yaw_0 = z_0[2];	double yaw_f = z_f[2];
	double x_s, y_s, yaw_s;

	DubinsSteer::SteerData steer_data;
	std::vector<std::vector<double>> traj_point_wise;
	std::vector<double> traj_len_map;
	std::vector<double> configuration_f;
	configuration_f = GetConfigurationF(z_0, z_f);
	std::vector<std::vector<double> > traj_desc(3, std::vector<double>(7, INFINITY));

	OutputData data_LSL = DubinLSL(configuration_f, radius_L, radius_R);
	OutputData data_RSR = DubinRSR(configuration_f, radius_L, radius_R);
	OutputData data_LSR = DubinLSR(configuration_f, radius_L, radius_R);
	OutputData data_RSL = DubinRSL(configuration_f, radius_L, radius_R);
	OutputData data_LRL = DubinLRL(configuration_f, radius_L, radius_R);
	OutputData data_RLR = DubinRLR(configuration_f, radius_L, radius_R);
	std::vector<double> all_length = {data_LSL.length, data_RSR.length, data_LSR.length, data_RSL.length, data_LRL.length, data_RLR.length};
	double min_length = *std::min_element(all_length.begin(), all_length.end());
	int min_pos = distance(all_length.begin(),std::min_element(all_length.begin(),all_length.end()));
	switch(min_pos){
		case 0:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 0;	traj_desc[2][0] = 1;
		traj_desc[0][3] = -1;	traj_desc[0][4] = radius_L;	traj_desc[0][6] = data_LSL.a;
		traj_desc[1][4] = data_LSL.s;
		traj_desc[2][3] = -1; 	traj_desc[2][4] = radius_L;	traj_desc[2][6] = data_LSL.g;
		break;
		case 1:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 0;	traj_desc[2][0] = 1;
		traj_desc[0][3] = 1;	traj_desc[0][4] = radius_R;	traj_desc[0][6] = data_RSR.a;
		traj_desc[1][4] = data_RSR.s;
		traj_desc[2][3] = 1; 	traj_desc[2][4] = radius_R;	traj_desc[2][6] = data_RSR.g;
		break;
		case 2:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 0;	traj_desc[2][0] = 1;
		traj_desc[0][3] = -1;	traj_desc[0][4] = radius_L;	traj_desc[0][6] = data_LSR.a;
		traj_desc[1][4] = data_LSR.s;
		traj_desc[2][3] = 1; 	traj_desc[2][4] = radius_R;	traj_desc[2][6] = data_LSR.g;
		break;
		case 3:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 0;	traj_desc[2][0] = 1;
		traj_desc[0][3] = 1;	traj_desc[0][4] = radius_R;	traj_desc[0][6] = data_RSL.a;
		traj_desc[1][4] = data_RSL.s;
		traj_desc[2][3] = -1; 	traj_desc[2][4] = radius_L;	traj_desc[2][6] = data_RSL.g;
		break;
		case 4:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 1;	traj_desc[2][0] = 1;
		traj_desc[0][3] = -1;	traj_desc[0][4] = radius_L;	traj_desc[0][6] = data_LRL.a;
		traj_desc[1][3] = 1;	traj_desc[1][4] = radius_R;	traj_desc[1][6] = data_LRL.b;
		traj_desc[2][3] = -1; 	traj_desc[2][4] = radius_L;	traj_desc[2][6] = data_LRL.g;
		break;
		case 5:
		traj_desc[0][0] = 1;	traj_desc[1][0] = 1;	traj_desc[2][0] = 1;
		traj_desc[0][3] = 1;	traj_desc[0][4] = radius_R;	traj_desc[0][6] = data_RLR.a;
		traj_desc[1][3] = -1;	traj_desc[1][4] = radius_L;	traj_desc[1][6] = data_RLR.b;
		traj_desc[2][3] = 1; 	traj_desc[2][4] = radius_R;	traj_desc[2][6] = data_RLR.g;
		break;
	}

	//for debug
	// for (int i = 0; i < 3; i++) {
	// 	for (int j = 0; j < 7; j++) {
	// 		std::cout << traj_desc[i][j] << ", "; 
	// 	}
	// 	std::cout << std::endl;
	// }

	for (int m1 = 0; m1 < 3; m1++){
		if (!traj_desc[m1][0]) continue;
		if ((traj_desc[m1][5]==INFINITY)&&(std::abs(traj_desc[m1][6] - 2*M_PI) < 1e-6)){
			traj_desc[m1][6] = 0;
		}
	}

	std::vector<double> traj_lengths(3,0);
	for (int np = 0; np < 3; np++){
		std::vector<double> t_spec = traj_desc[np];
		int temp = t_spec[0];
		switch(temp){
			case 0:
			traj_lengths[np] = t_spec[4];
			break;
			case 1:
			traj_lengths[np] = std::abs(t_spec[6])*t_spec[4];
			break;
		}
	}

	int num_points_1 = 60;
	double step_length = *std::max_element(traj_lengths.begin(), traj_lengths.end())/num_points_1;
	int num_points_2 = std::ceil(traj_lengths[0]/step_length)+std::ceil(traj_lengths[1]/step_length)+std::ceil(traj_lengths[2]/step_length);
	int num_points_3 = 0;
	int num_points_4 = 0;
	std::vector<int> num_indx_range;
	double travLength = 0.0;
	for (int np = 0; np < 3; np++){
		std::vector<double> t_spec = traj_desc[np];
		int temp_x = t_spec[0];
		switch(temp_x){

			case 0:{
				if (traj_lengths[np]==0){
					break;
				}
				if (t_spec[1]==INFINITY){
					x_s = x_0;
				}
				else {
					x_s = t_spec[1];
				}
				if (t_spec[2]==INFINITY){
					y_s = y_0;
				}
				else {
					y_s = t_spec[2];
				}
				if (t_spec[3]==INFINITY){
					yaw_s = yaw_0;
				}
				else {
					yaw_s = t_spec[3];
				}
				double L = t_spec[4];
				
				int range_up = num_points_3 + std::ceil(traj_lengths[np]/step_length);
				for (int it = num_points_3; it < range_up; it++){
					num_indx_range.push_back(it);
				}
				if (num_indx_range.empty()) {
					num_points_4 = 0;
				}
				else {
					num_points_4 = num_indx_range.back() + 1 - num_points_3;
				}
				std::vector<double> a_par = LinSpace(0, L, num_points_4);
				for (int i = 0; i < a_par.size(); i++){
					std::vector<double> temp = {x_s + a_par[i]*cos(yaw_s), y_s + a_par[i]*sin(yaw_s), yaw_s};
					traj_point_wise.push_back(temp);
					double temp_len = travLength + L*(i - num_points_3)/num_points_4;
					traj_len_map.push_back(temp_len);
				}

				if (num_indx_range.empty()) {
					num_points_3 = 0;
				}
				else {
					num_points_3 = num_indx_range.back() + 1;
				}
				x_0 = x_0 + L*cos(yaw_s);
				y_0 = y_0 + L*sin(yaw_s);
				yaw_0 = yaw_s;
				travLength = travLength + L;
				break;
			}
			case 1:{
				if (traj_lengths[np]==0){
					break;
				}
				double dirn = t_spec[3];
				double r_crv = t_spec[4];
				if (t_spec[1]==INFINITY){
					x_s = x_0;
				}
				else{
					x_s = t_spec[1];
				}
				if (t_spec[2]==INFINITY){
					y_s = y_0;
				}
				else {
					y_s = t_spec[2];
				}
				if (t_spec[5]==INFINITY){
					yaw_s = yaw_0;
				}
				else {
					yaw_s = t_spec[5];
				}
				if (t_spec[5]==INFINITY){
					yaw_f = yaw_0 - dirn*t_spec[6];
				}
				else{
					yaw_f = t_spec[6];
				}
				double x_c = x_s + dirn*r_crv*sin(yaw_s);
				double y_c = y_s - dirn*r_crv*cos(yaw_s);

				int range_up = num_points_3 + std::ceil(traj_lengths[np]/step_length);				
				for (int it = num_points_3; it < range_up; it++){
					num_indx_range.push_back(it);
				}

				if (num_indx_range.empty()) {
					num_points_4 = 0;
				}
				else {
					num_points_4 = num_indx_range.back() + 1 - num_points_3;
				}
				std::vector<double> a_par = LinSpace(yaw_s + dirn*M_PI/2, yaw_f + dirn*M_PI/2, num_points_4);
				for (int i = 0; i < a_par.size(); i++){
					std::vector<double> temp = {x_c + r_crv*cos(a_par[i]), y_c + r_crv*sin(a_par[i]), a_par[i] - dirn*M_PI/2};
					traj_point_wise.push_back(temp);
					double temp_len = travLength + std::abs(yaw_f - yaw_s)*r_crv*(i - num_points_3)/num_points_4;
					traj_len_map.push_back(temp_len);
				}
				if (num_indx_range.empty()) {
					num_points_3 = 0;
				}
				else {
					num_points_3 = num_indx_range.back() + 1;
				}
				x_0 = x_c + r_crv*cos(yaw_f + dirn*M_PI/2);
				y_0 = y_c + r_crv*sin(yaw_f + dirn*M_PI/2);
				yaw_0 = yaw_f;
				travLength = travLength + std::abs(yaw_f - yaw_s)*r_crv;
				break;
			}
		}
		
	}
	steer_data.traj_length = min_length;
	steer_data.traj_point_wise = traj_point_wise;
	steer_data.traj_len_map = traj_len_map;
	return steer_data;
}
