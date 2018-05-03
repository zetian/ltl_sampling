/*
 * test_calc.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <tuple>
#include <memory>

// opencv
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "geometry/square_grid.h"
#include "map/map_utils.h"
#include "vis/sgrid_vis.h"
#include "vis/graph_vis.h"

using namespace librav;

class MapHandler 
{
	public:
		MapHandler(std::shared_ptr<lcm::LCM> lcm):
			lcm_(lcm),map_received_(false) {}
		~MapHandler(){}
		
		std::shared_ptr<lcm::LCM> lcm_;
		bool map_received_;
		std::shared_ptr<SquareGrid> sgrid_;

		void sendMapRequest()
		{
			librav_lcm_msgs::MapRequest_t map_rqt_msg;
			map_rqt_msg.new_map_requested = true;
			map_rqt_msg.map_size_x = 15;
			map_rqt_msg.map_size_y = 30;
			map_rqt_msg.map_size_z = 5;
			map_rqt_msg.map_type = 2;
			lcm_->publish("envsim/map_request", &map_rqt_msg);
			std::cout << "New map request sent" << std::endl;
		}

        void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::Map_t *msg)
        {
			map_received_ = true;

			// otherwise process the msg to get a new map
			std::cout << "Map msg received: " << std::endl;
			std::cout << "Map size: " << msg->cell_num << std::endl;

			// create square grid from map msg
			double side_size = 10.0;
			sgrid_ = MapUtils::CreateSquareGrid(msg->size_x, msg->size_y, side_size);
			for (const auto &cell : msg->cells)
			{
				if (cell.occupied)
				{
					sgrid_->SetCellOccupancy(cell.pos_x, cell.pos_y, OccupancyType::OCCUPIED);
				}
			}

			cv::Mat vis_img;
			Vis::VisSquareGrid(*sgrid_, vis_img);
			cv::namedWindow("Processed Image", cv::WINDOW_NORMAL);
			cv::imshow("Processed Image", vis_img);
			cv::imwrite("map.png",vis_img);
			cv::waitKey(0);

			sendMapRequest();
        }
};

int main(int argc, char *argv[])
{

	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if (!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	MapHandler handler(lcm);
	lcm->subscribe("envsim/map", &MapHandler::handleMessage, &handler);

	handler.sendMapRequest();

	while(true)
	{
		lcm->handleTimeout(0);
	}

	return 0;
}
