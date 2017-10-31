#include "dubins_path.h"
#include <math.h>
#include <assert.h>
#include <iostream>

#define EPSILON (10e-10)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

// The segment types for each of the Path types
const int DIRDATA[6][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

double fmodr(double x, double y){
    return x - y*floor(x/y);
}

double mod2pi(double theta){
    return fmodr(theta, 2*M_PI);
}


int dubins_LSL(double alpha, double beta, double d, std::vector<double>& outputs){
    double tmp0 = d + sin(alpha) - sin(beta);
    double p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
    if( p_squared < 0 ){
        return EDUBNOPATH;
    }
    double tmp1 = atan2( (cos(beta) - cos(alpha)), tmp0);
    double t = mod2pi(-alpha + tmp1);
    double p = sqrt(p_squared);
    double q = mod2pi(beta - tmp1);
    outputs = {t, p, q};
    return EDUBOK;
}

int dubins_RSR(double alpha, double beta, double d, std::vector<double>& outputs){
    double tmp0 = d - sin(alpha) + sin(beta);
    double p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta) - sin(alpha)));
    if( p_squared < 0 ){
        return EDUBNOPATH;
    }
    double tmp1 = atan2((cos(alpha) - cos(beta)), tmp0);
    double t = mod2pi(alpha - tmp1);
    double p = sqrt(p_squared);
    double q = mod2pi(-beta + tmp1);
    outputs = {t, p, q};
    return EDUBOK;
}

int dubins_LSR(double alpha, double beta, double d, std::vector<double>& outputs){
    double p_squared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha) + sin(beta)));
    if( p_squared < 0 ){
        return EDUBNOPATH;
    }
    double p = sqrt(p_squared);
    double tmp2 = atan2( (-cos(alpha) - cos(beta)), (d+sin(alpha) + sin(beta)) ) - atan2(-2.0, p);
    double t = mod2pi(-alpha + tmp2);
    double q = mod2pi(-mod2pi(beta) + tmp2);
    outputs = {t, p, q};
    return EDUBOK;
}

int dubins_RSL(double alpha, double beta, double d, std::vector<double>& outputs){
    double p_squared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha) + sin(beta)));
    if( p_squared< 0 ) {
        return EDUBNOPATH;
    }
    double p = sqrt(p_squared);
    double tmp2 = atan2((cos(alpha) + cos(beta)), (d-sin(alpha) - sin(beta)) ) - atan2(2.0, p);
    double t = mod2pi(alpha - tmp2);
    double q = mod2pi(beta - tmp2);
    outputs = {t, p, q};
    return EDUBOK;
}

int dubins_RLR(double alpha, double beta, double d, std::vector<double>& outputs){
    double tmp_rlr = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha) - sin(beta)))/8.;
    if( fabs(tmp_rlr) > 1){
        return EDUBNOPATH;
    }
    double p = mod2pi(2*M_PI - acos(tmp_rlr));
    double t = mod2pi(alpha - atan2( cos(alpha) - cos(beta), d-sin(alpha) + sin(beta)) + mod2pi(p/2.));
    double q = mod2pi(alpha - beta - t + mod2pi(p));
    outputs = {t, p, q};
    return EDUBOK;
}

int dubins_LRL(double alpha, double beta, double d, std::vector<double>& outputs){
    double tmp_lrl = (6. - d*d + 2*cos(alpha - beta) + 2*d*(- sin(alpha) + sin(beta))) / 8.;
    if( fabs(tmp_lrl) > 1) {
        return EDUBNOPATH;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_lrl ) );
    double t = mod2pi(-alpha - atan2( cos(alpha) - cos(beta), d+sin(alpha) - sin(beta) ) + p/2.);
    double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
    outputs = {t, p, q};
    return EDUBOK;
}

double dubins_path_length(DubinsPathInfo& path){
    double length = 0.;
    length += path.param[0];
    length += path.param[1];
    length += path.param[2];
    length = length*path.rho;
    return length;
}

int dubins_curve_type(int type, double alpha, double beta, double d, std::vector<double>& outputs){
    switch(type){
        case 0: return dubins_LSL(alpha, beta, d, outputs);
        case 1: return dubins_LSR(alpha, beta, d, outputs);
        case 2: return dubins_RSL(alpha, beta, d, outputs);
        case 3: return dubins_RSR(alpha, beta, d, outputs);
        case 4: return dubins_RLR(alpha, beta, d, outputs);
        case 5: return dubins_LRL(alpha, beta, d, outputs);
    }
    return 0;
}

int dubins_init_normalised(double alpha, double beta, double d, DubinsPathInfo& path){
    double best_cost = INFINITY;
    int best_word = -1;
    for(int i = 0; i < 6; i++) {
        std::vector<double> params;
        int err = dubins_curve_type(i, alpha, beta, d, params);
        if(err == EDUBOK) {
            double cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path.param = params;
                path.type = i;
            }
        }
    }
    if(best_word == -1) {
        return EDUBNOPATH;
    }
    path.type = best_word;
    return EDUBOK;
}

int dubins_init(std::vector<double> q0, std::vector<double> q1, double rho, DubinsPathInfo& path){
    int i;
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);
    for( i = 0; i < 3; i ++ ) {
        path.qi[i] = q0[i];
    }
    path.rho = rho;
    return dubins_init_normalised(alpha, beta, d, path);
}

int dubins_path_type(DubinsPathInfo& path){
    return path.type;
}

void dubins_segment(double t, std::vector<double> qi, std::vector<double>& qt, int type){
    assert(type == L_SEG || type == S_SEG || type == R_SEG);
    if(type == L_SEG){
        qt = {qi[0] + sin(qi[2]+t) - sin(qi[2]), qi[1] - cos(qi[2]+t) + cos(qi[2]), qi[2] + t};
    }
    else if(type == R_SEG){
        qt = {qi[0] - sin(qi[2]-t) + sin(qi[2]), qi[1] + cos(qi[2]-t) - cos(qi[2]), qi[2] - t};
    }
    else if(type == S_SEG){
        qt = {qi[0] + cos(qi[2])*t, qi[1] + sin(qi[2])*t, qi[2]};
    }
}

std::vector<double> dubins_path_sample(DubinsPathInfo& path, double t){
    std::vector<double> q;
    // tprime is the normalised variant of the parameter t
    double tprime = t / path.rho;
    // The translated initial configuration
    std::vector<double> qi = {0, 0, path.qi[2]};
    // Generate the target configuration
    const int* types = DIRDATA[path.type];
    double p1 = path.param[0];
    double p2 = path.param[1];
    std::vector<double> q1;
    std::vector<double> q2;
    dubins_segment(p1, qi, q1, types[0]);
    dubins_segment(p2, q1, q2, types[1]);
    if(tprime < p1){
        dubins_segment(tprime, qi, q, types[0]);
    }
    else if(tprime < (p1+p2)){
        dubins_segment(tprime-p1, q1, q, types[1]);
    }
    else {
        dubins_segment(tprime-p1-p2, q2, q, types[2]);
    }
    // scale the target configuration, translate back to the original starting point
    q = {q[0] * path.rho + path.qi[0], q[1] * path.rho + path.qi[1], mod2pi(q[2])};
    return q;
}

DubinsPath::PathData dubins_path_sample_many(DubinsPathInfo& path, double stepSize){
    DubinsPath::PathData dubins_steer_data;
    double x = 0.0;
    double length = dubins_path_length(path);
    dubins_steer_data.traj_length = length;
    while(x <  length){
        std::vector<double> q = dubins_path_sample(path, x);
        dubins_steer_data.traj_point_wise.push_back(q);
        dubins_steer_data.traj_len_map.push_back(x);
        x += stepSize;
    }
    return dubins_steer_data;
}

std::vector<double> dubins_path_endpoint(DubinsPathInfo& path)
{
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON);
}

int dubins_extract_subpath(DubinsPathInfo& path, double t, DubinsPathInfo& newpath){
    // calculate the true parameter
    double tprime = t / path.rho;

    // copy most of the data
    newpath.qi[0] = path.qi[0];
    newpath.qi[1] = path.qi[1];
    newpath.qi[2] = path.qi[2];
    newpath.rho   = path.rho;
    newpath.type  = path.type;

    // fix the parameters
    newpath.param[0] = fmin( path.param[0], tprime );
    newpath.param[1] = fmin( path.param[1], tprime - newpath.param[0]);
    newpath.param[2] = fmin( path.param[2], tprime - newpath.param[0] - newpath.param[1]);
    return 0;
}

DubinsPath::PathData DubinsPath::GetDubinsPathPointWise(std::vector<double> q0, std::vector<double> q1, double min_radius, double step){
    DubinsPathInfo path;
    dubins_init(q0, q1, min_radius, path);
    DubinsPath::PathData path_data;
    path_data = dubins_path_sample_many(path, step);
    return path_data;
}

double DubinsPath::GetDubinsPathLength(std::vector<double> q0, std::vector<double> q1, double min_radius){
    DubinsPathInfo path;
    dubins_init(q0, q1, min_radius, path);
    return dubins_path_length(path);
}




