/*
 * octomap_server_node.cpp
 *
 *  Created on: May 27, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <ctime>
#include <chrono>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include "map/octomap_server.h"

using namespace librav;

int main(int argc, char** argv)
{
	double octree_reso = 0.341151;
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	std::string octomap_file_name = "octree_server_saved_map.bt";
	if ( argc == 2 ) {
		std::string file_name = argv[1];
		octomap_file_name = file_name + ".bt";
	}
	else if ( argc == 3 ) {
		std::string res = argv[1];
		octree_reso = stof(res);

		std::string file_name = argv[2];
		octomap_file_name = file_name + ".bt";
	}

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return 1;
	}

	OctomapServer server(lcm);
	server.SetOctreeResolution(octree_reso);

	std::cout << "INFO: Octomap server started." << std::endl;

	bool tree_saved = false;
	clock_t start_time;
	start_time = clock();

	while(true)
	{
		lcm->handleTimeout(0);

		double duration =  double(clock() - start_time)/CLOCKS_PER_SEC;
		if(duration > 10 && !tree_saved) {
			server.SaveTreeToFile(octomap_file_name);
			tree_saved = true;
		}

//		 delay for some time
//		std::chrono::seconds timespan(10); // or whatever
//		std::this_thread::sleep_for(timespan);
	}

	return 0;
}
