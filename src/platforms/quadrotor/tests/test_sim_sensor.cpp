/*
 * test_sim_sensor.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: rdu
 */

#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "path_repair/sim/sim_depth_sensor.cpp"
#include "geometry/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace librav;

int main()
{
    SimDepthSensor sensor;

    sensor.SetRange(8);
    sensor.SetFOV(M_PI / 3.0 * 2.0);

    auto ws = CubeArrayBuilder::BuildEmptyCubeArray(20, 20, 20, 1.0);
    sensor.SetWorkspace(ws, 1.0);

    auto area = sensor.GetSensedArea(10, 10, 10, -M_PI/2.0);

    ////////////////////////////////////////////////////////////////
    // send data to LCM network for visualization
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

    if (!lcm->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    std::shared_ptr<Graph_t<CubeCell &>> cubegraph = GraphBuilder::BuildFromCubeArray(area);

    srcl_lcm_msgs::Graph_t graph_msg;

    graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
    for (auto &vtx : cubegraph->GetGraphVertices())
    {
        srcl_lcm_msgs::Vertex_t vertex;
        vertex.id = vtx->vertex_id_;

        vertex.position[0] = vtx->bundled_data_.location_.x;
        vertex.position[1] = vtx->bundled_data_.location_.y;
        vertex.position[2] = vtx->bundled_data_.location_.z;
        graph_msg.vertices.push_back(vertex);
    }

    graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
    for (auto &eg : cubegraph->GetGraphUndirectedEdges())
    {
        srcl_lcm_msgs::Edge_t edge;
        edge.id_start = eg.src_->vertex_id_;
        edge.id_end = eg.dst_->vertex_id_;

        graph_msg.edges.push_back(edge);
    }

    //std::cout << "vertex size: " << graph_msg.vertex_num << " , edge size: " << graph_msg.edge_num << std::endl;

    lcm->publish("quad_planner/geo_mark_graph", &graph_msg);

    std::cout << "######################## graph sent ########################" << std::endl;

    return 0;
}