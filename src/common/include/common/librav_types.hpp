/* 
 * librav_types.hpp
 * 
 * Created on: Nov 06, 2017 11:33
 * Description: common type definitions for librav, evolved from 
 *              planning_types.h and control_types.h   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef LIBRAV_TYPES_HPP
#define LIBRAV_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Core"

namespace librav
{

// time_stamp starts from 0 when system initialized, increases at step 1 ms
typedef uint64_t time_stamp;

template <typename T>
struct value2d
{
	value2d() : x(0), y(0) {}
	value2d(T _x, T _y) : x(_x), y(_y) {}

	T x;
	T y;

	bool operator==(const struct value2d &other) const
	{
		if (this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value2d &pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
};

template <typename T>
struct value3d
{
	value3d() : x(0), y(0), z(0) {}
	value3d(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	T x;
	T y;
	T z;

	bool operator==(const struct value3d &other) const
	{
		if (this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value3d &pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
};

using Point3f = value3d<float>;
using Point3d = value3d<double>;
using Point3i = value3d<int32_t>;

using Position2Di = value2d<int32_t>;
using Position2Dd = value2d<double>;
using Position3Di = value3d<int32_t>;
using Position3Dd = value3d<double>;

using Velocity2Di = value2d<int32_t>;
using Velocity2Dd = value2d<double>;
using Velocity3Di = value3d<int32_t>;
using Velocity3Dd = value3d<double>;

/****************** Types for Control ******************/

struct EulerAngle
{
	float roll;
	float pitch;
	float yaw;
};

struct Quaternion
{
	float x;
	float y;
	float z;
	float w;
};

struct Pose
{
	Point3f pos;
	EulerAngle ori;
};

struct UAVTrajectoryPoint
{
	bool point_empty;
	float positions[3];
	float velocities[3];
	float accelerations[3];
	float jerks[3];
	float yaw;
	float yaw_rate;
	uint64_t duration; // in milliseconds
};

typedef std::vector<UAVTrajectoryPoint> UAVTrajectory;

/****************** Types for Planning ******************/

enum class OccupancyType
{
	FREE,
	OCCUPIED,
	// only above two are used for a graph
	MIXED,
	INTERESTED,
	UNKONWN,
	EXPANDED_OBS
};

template<typename T>
struct Range2D
{
	T min;
	T max;
};

template<typename T>
struct BoundingBox
{
	Range2D<T> x;
	Range2D<T> y;
};

struct Keyframe
{
	float position[3];
	float velocity[3];
	float yaw;

	bool pos_constr;
	bool vel_constr;
	bool yaw_constr;
};

struct KeyframeSet
{
	std::vector<Keyframe> keyframes;
	uint64_t start_time;
};

/****************** Types for Sensors ******************/
// Deprecated
struct IMUData
{
	time_stamp mtime;
	Point3f gyro;
	Point3f acc;
};

struct AccGyroData
{
	AccGyroData():
		mtime(0),
		accel(Point3d(0,0,0)), 
		gyro(Point3d(0,0,0)){};

	AccGyroData(int64_t time, double accel_x, double accel_y, double accel_z,
		double gyro_x, double gyro_y, double gyro_z):
		mtime(time),
		accel(Point3d(accel_x,accel_y,accel_z)), 
		gyro(Point3d(gyro_x,gyro_y,gyro_z)){};

	int64_t mtime;
	Point3d accel;
	Point3d gyro;

	friend std::ostream &operator<<(std::ostream &os, const AccGyroData &data)
	{
		os << "time_stamp: " << data.mtime << " ; accel(x,y,z): " << data.accel.x << " , " << data.accel.y << " , " << data.accel.z
			<< " ; gyro(x,y,z): " << data.gyro.x << " , " << data.gyro.y << " , " << data.gyro.z << std::endl;
		return os;
	}
};

struct CarSpeed
{
	CarSpeed():
		mtime(0),
		speed(0.0){};

	CarSpeed(int64_t time, float spd):
		mtime(time),
		speed(spd){};

	int64_t mtime;
	float speed;

	friend std::ostream &operator<<(std::ostream &os, const CarSpeed &data)
	{
		os << "time_stamp: " << data.mtime << " ; speed: " << data.speed << std::endl;
		return os;
	}
};

struct IMUCalibParams
{
    Eigen::Matrix<double,3,3> misalignment_matrix;
    Eigen::Matrix<double,3,3> scale_matrix;
    Eigen::Matrix<double,3,1> bias_vector;
};

}

#endif /* LIBRAV_TYPES_H */
