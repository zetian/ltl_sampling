/*
 * vertex.h
 *
 *  Created on: Feb 1, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_VERTEX_H_
#define SRC_GRAPH_VERTEX_H_

#include <cstdint>
#include <algorithm>

#include "graph/edge.h"

namespace srcl {

/****************************************************************************/
/*								 Vertex										*/
/****************************************************************************/
/// A vertex data structure template.
template<typename BundledStructType>
class Vertex
{
public:
	template<class T = BundledStructType, typename std::enable_if<std::is_pointer<T>::value>::type* = nullptr>
	Vertex(BundledStructType bundled_data):
		// attributes related to associated node
		bundled_data_(bundled_data),vertex_id_(bundled_data->data_id_),
		product_vertex_id_(0),
		// common attributes
		search_parent_(nullptr),
		is_checked_(false), is_in_openlist_(false),
		f_astar_(0),g_astar_(0),h_astar_(0){};

	template<class T = BundledStructType, typename std::enable_if<!std::is_pointer<T>::value>::type* = nullptr>
	Vertex(BundledStructType bundled_data):
		// attributes related to associated node
		bundled_data_(bundled_data), vertex_id_(bundled_data.data_id_),
		product_vertex_id_(0),
		// common attributes
		search_parent_(nullptr),
		is_checked_(false), is_in_openlist_(false),
		f_astar_(0),g_astar_(0),h_astar_(0){};

	~Vertex(){
		edges_.clear();
	};

	// friends
	template<typename BDSType>
	friend class Graph;
	friend class AStar;

	// generic attributes
	BundledStructType bundled_data_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex<BundledStructType>*>> edges_;
	std::vector<uint64_t> lifted_vertices_id_;
	uint64_t product_vertex_id_;

private:
    // attributes for A* search
	bool is_checked_;
	bool is_in_openlist_;
	double f_astar_;
	double g_astar_;
	double h_astar_;
	Vertex<BundledStructType>* search_parent_;

private:
	/// Clear exiting search info before a new search
	void ClearVertexSearchInfo()
	{
		is_checked_ = false;
		is_in_openlist_ = false;
		search_parent_ = nullptr;

		f_astar_ = 0.0;
		g_astar_ = 0.0;
		h_astar_ = 0.0;
	}

	/// Get heuristic using function provided by bundled data (pointer type)
	template<class T = BundledStructType, typename std::enable_if<std::is_pointer<T>::value>::type* = nullptr>
	double CalcHeuristic(Vertex<T>* dst_vertex)
	{
		return this->bundled_data_->GetHeuristic(*(dst_vertex->bundled_data_));
	}

	/// Get heuristic using function provided by bundled data (non-pointer type)
	template<class T = BundledStructType,
			typename std::enable_if<!std::is_pointer<T>::value>::type* = nullptr>
	double CalcHeuristic(Vertex<T>* dst_vertex)
	{
		return this->bundled_data_.GetHeuristic(dst_vertex->bundled_data_);
	}

public:
	/// == operator overloading. If two vertices have the same id, they're regarded as equal.
	bool operator ==(const Vertex<BundledStructType>& other) const
	{
		if(vertex_id_ == other.vertex_id_)
			return true;
		else
			return false;
	}

	/// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
	///		the two vertices exists.
	double GetEdgeCost(const Vertex<BundledStructType>& dst_node) const
	{
		double cost = -1;

		for(const auto& it : edges_)
		{
			if(it.dst_.vertex_id_ == dst_node.vertex_id_)
			{
				cost = it.cost_;
				break;
			}
		}

		return cost;
	}

	std::vector<uint32_t> GetBuchiTransition(Vertex<BundledStructType>* dst_node){
		std::vector<uint32_t> buchi_transition;
		for(auto& it : edges_)
		{
			if(it.dst_->vertex_id_ == dst_node->vertex_id_)
			{
				buchi_transition = it.buchi_transition_;
				break;
			}
		}
		return buchi_transition;
	}


	/// Get all neighbor vertices of this vertex.
	std::vector<Vertex<BundledStructType>*> GetNeighbours()
	{
		std::vector<Vertex<BundledStructType>*> neighbours;

		for(const auto& edge:edges_)
			neighbours.push_back(edge.dst_);

		return neighbours;
	}

	/// Check if a given vertex is the neighbor of current vertex.
	bool CheckNeighbour(Vertex<BundledStructType>* dst_node)
	{
		std::vector<Vertex<BundledStructType>*> neighbours = GetNeighbours();

		auto it = find(neighbours.begin(), neighbours.end(), dst_node);

		if(it != neighbours.end())
			return true;
		else
			return false;
	}
};

}

#endif /* SRC_GRAPH_VERTEX_H_ */
