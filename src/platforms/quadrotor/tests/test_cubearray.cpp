/*
 * test_cubearray.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <iostream>

#include "geometry/cube_array/cube_array.h"

using namespace librav;

int main(int argc, char* argv[])
{
//	CubeArray ca(3,3,3,0.5);
//	ca.SetOriginOffset(1,1,0);

//	size: 13 , 9 , 5
//	offset: 0 , 5 , 4
	CubeArray ca(13, 9, 5, 0.3);
	ca.SetOriginOffset(0,4,4);

	std::cout << "\n----------------------------\n" << std::endl;
//	std::cout << "id: " << ca.GetIDFromPosition(1.2, 1.2, 0.3) << std::endl;
//	std::cout << "id: " << ca.GetIDFromPosition(0.45, 0.45, 0.3) << std::endl;
//	std::cout << "id: " << ca.GetIDFromPosition(-0.2, 0.2, 0.3) << std::endl;
//	std::cout << "id: " << ca.GetIDFromPosition(0.2, -0.2, 0.3) << std::endl;
//	std::cout << "id: " << ca.GetIDFromPosition(0.2, 0.8, 0.3) << std::endl;

//	for(auto& nd : ca.GetNeighbours(13))
//		std::cout << "neighbour: " << nd << std::endl;
}


