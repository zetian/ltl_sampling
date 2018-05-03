/*
 * test_trajdef.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <iostream>
#include <vector>

#include "common/poly_curve.h"
#include "common/quad_flattraj.h"

using namespace librav;

int main(int argc, char* argv[])
{
	std::vector<double> coeffs;

	coeffs.push_back(2);
	coeffs.push_back(3);
	coeffs.push_back(1);

	PolyCurve seg(coeffs, true, 0, 1);

//	std::cout << "point value: " << seg.GetTrajPointDerivVal(1, 0) << std::endl;
//	std::cout << "point value: " << seg.GetTrajPointDerivVal(1, 0.5) << std::endl;
//	std::cout << "point value: " << seg.GetTrajPointDerivVal(1, 1) << std::endl;

//	std::cout << "pos: " << seg.GetTrajPointPos(-0.1) << std::endl;
//	std::cout << "pos: " << seg.GetTrajPointPos(0) << std::endl;
//	std::cout << "pos: " << seg.GetTrajPointPos(0.5) << std::endl;
//	std::cout << "pos: " << seg.GetTrajPointPos(1) << std::endl;
//	std::cout << "pos: " << seg.GetTrajPointPos(1.2) << std::endl;
//
//	std::cout << "vel: " << seg.GetTrajPointVel(-0.1) << std::endl;
//	std::cout << "vel: " << seg.GetTrajPointVel(0) << std::endl;
//	std::cout << "vel: " << seg.GetTrajPointVel(0.5) << std::endl;
//	std::cout << "vel: " << seg.GetTrajPointVel(1) << std::endl;
//	std::cout << "vel: " << seg.GetTrajPointVel(1.2) << std::endl;
//
//	std::cout << "acc: " << seg.GetTrajPointAcc(-0.1) << std::endl;
//	std::cout << "acc: " << seg.GetTrajPointAcc(0) << std::endl;
//	std::cout << "acc: " << seg.GetTrajPointAcc(0.5) << std::endl;
//	std::cout << "acc: " << seg.GetTrajPointAcc(1) << std::endl;
//	std::cout << "acc: " << seg.GetTrajPointAcc(1.2) << std::endl;

	QuadFlatTraj traj;
}
