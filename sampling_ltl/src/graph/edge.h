/*
 * edge.h
 *
 *  Created on: Jul 14, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_GRAPH_EDGE_H_
#define PLANNING_SRC_GRAPH_EDGE_H_

#include <iostream>

namespace srcl {

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/
/// An edge data structure template.
template<typename VertexType>
class Edge
{
public:
	/**
	 * @param src a pointer to the source vertex of the edge
	 * @param dst a pointer to the destination vertex of the edge
	 * @param c cost associated with the edge
	 */
	Edge(VertexType src, VertexType dst, double c = 0.0):
		src_(src),dst_(dst), cost_(c){};
	~Edge(){};

	VertexType src_;
	VertexType dst_;
	double cost_;
	std::vector<uint32_t> buchi_transition_;

	/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
	bool operator ==(const Edge<VertexType>& other)
	{
		if(src_.vertex_id_ == other.src_.vertex_id_ && dst_.vertex_id_ == other.dst_.vertex_id_)
			return true;
		else
			return false;
	}

	/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
	bool operator -=(const Edge<VertexType>& other)
	{
		if((src_.vertex_id_ == other.src_.vertex_id_ && dst_.vertex_id_ == other.dst_.vertex_id_)
				|| (src_.vertex_id_ == other.dst_.vertex_id_ && dst_.vertex_id_ == other.src_.vertex_id_))
			return true;
		else
			return false;
	}

	/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
	void PrintEdge() const
	{
		std::cout << "Edge: src - " << src_.vertex_id_ << " , dst - " << dst_.vertex_id_ << " , cost - " << cost_ << std::endl;
	}
};

// Partial specialization of the Edge class template for pointer types
template<typename VertexType>
class Edge<VertexType*>
{
public:
	/**
	 * @param src a pointer to the source vertex of the edge
	 * @param dst a pointer to the destination vertex of the edge
	 * @param c cost associated with the edge
	 */
	Edge(VertexType* src, VertexType* dst, double c = 0.0):
		src_(src),dst_(dst), cost_(c){};
	~Edge(){};

	VertexType* src_;
	VertexType* dst_;
	double cost_;
	std::vector<uint32_t> buchi_transition_;

	/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
	bool operator ==(const Edge<VertexType*>& other)
	{
		if(src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_)
			return true;
		else
			return false;
	}

	/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
	bool operator -=(const Edge<VertexType*>& other)
	{
		if((src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_)
				|| (src_->vertex_id_ == other.dst_->vertex_id_ && dst_->vertex_id_ == other.src_->vertex_id_))
			return true;
		else
			return false;
	}

	/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
	void PrintEdge() const
	{
		std::cout << "Edge: src - " << src_->vertex_id_ << " , dst - " << dst_->vertex_id_ << " , cost - " << cost_ << std::endl;
	}
};

}

#endif /* PLANNING_SRC_GRAPH_EDGE_H_ */
