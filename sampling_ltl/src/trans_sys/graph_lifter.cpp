#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>

#include "trans_sys/graph_lifter.h"

using namespace srcl;

namespace {
///This iterative function return histories of a vertex in a graph
std::vector<std::vector<Vertex_t<SquareCell*>*>> GetHistories_inner_iterative(Vertex_t<SquareCell*>* actNode, int historyH, int seqPos, std::vector<Vertex_t<SquareCell*>*>& rawSeq1, std::vector<std::vector<Vertex_t<SquareCell*>*>>& allHist){
	/****************************************************************************
	 * actNode: active node from which all histories start
	 * historyH: int value, the length of histories
	 * rawSeq1: input for propagating histories generated thus far to further recursive calls
	 *
	/****************************************************************************/
	bool termination;
	std::vector<Vertex_t<SquareCell*>*> rawSeq2;
	Vertex_t<SquareCell*>* actNhbr;
	std::vector<Vertex_t<SquareCell*>*> temp_seq;
	std::vector<Vertex_t<SquareCell*>*> allNhbr;
	///First vertex of history is a neighbour of the vertex
	allNhbr = actNode->GetNeighbours();
	for (auto it = allNhbr.begin(); it != allNhbr.end(); it++){
		termination = false;
		actNhbr =*it;
		///If it's the last vertex of a history, then this history is done
		if (seqPos == historyH - 2){
			///If the candidate is already a history, continue
			if (std::find(rawSeq1.begin(),rawSeq1.end(),actNhbr)!=rawSeq1.end()){
				continue;
			}
			///If the candidate is not a neighbour of a previous history, continue
			if (!rawSeq1.empty()){
				for(auto ite = rawSeq1.begin(); ite!=rawSeq1.end()-1; ite++){
					if (actNhbr->CheckNeighbour(*ite)){
						termination = true;
						continue;
					}
				}
			}
			temp_seq = rawSeq1;
			temp_seq.push_back(actNhbr);
			if (termination){
				continue;
			}
			///Push back a new history
			allHist.push_back(temp_seq);
		}
		///If it's not the last vertex of a history, keep call this function to find next vertex in history
		else{
			///Same, if the candidate is already a history, continue
			if (std::find(rawSeq1.begin(),rawSeq1.end(),actNhbr)!=rawSeq1.end()){
				continue;
			}
			///If the candidate is not a neighbour of a previous history, continue
			if (!rawSeq1.empty()){
				for(auto ite = rawSeq1.begin(); ite!=rawSeq1.end()-1; ite++){
					if (actNhbr->CheckNeighbour(*ite)){
						termination = true;
						continue;
					}
				}
			}
			if (termination){
				continue;
			}
			rawSeq2 = rawSeq1;
			rawSeq2.push_back(actNhbr);
			///Keep call this function to get next vertex in history
			allHist = GetHistories_inner_iterative(actNhbr, historyH, seqPos+1, rawSeq2, allHist);
		}
	}
	return allHist;
}
}

///Build a lifted graph from square cell type graph
std::shared_ptr<srcl::Graph_t<LiftedSquareCell>> GraphLifter::CreateLiftedGraph(int historyH, std::shared_ptr<Graph_t<SquareCell*>> original_graph)
{

	///Define a new empty lifted graph
	std::shared_ptr<srcl::Graph_t<LiftedSquareCell>> lifted_graph = std::make_shared<Graph_t<LiftedSquareCell>>();
	std::vector<Vertex<SquareCell*>*> all_verts_origin = original_graph->GetGraphVertices();
	std::vector<std::vector<Vertex<SquareCell*>*>> all_nodes_histories;
	std::vector<LiftedSquareCell> all_lifted_node;
	std::vector<std::vector<Vertex<SquareCell*>*>> history;
	///A vector use to searching neighbours in lifted graph
	std::vector<int> idx_hist;
	int last_idx;
	uint64_t Lifted_graph_id;
	Lifted_graph_id = 0;
	///Get history for all vertices in original graph
	idx_hist.push_back(0);
	for (auto it = all_verts_origin.begin();it!=all_verts_origin.end();it++){
		(*it)->lifted_vertices_id_.clear(); //clear the lifted id in original graph
		history.clear();
		history = GetHistories(*it,historyH);
		last_idx = idx_hist.back();
		idx_hist.push_back(last_idx+history.size());
		for (auto ite = history.begin();ite != history.end(); ite++){
			all_nodes_histories.push_back(*ite);
		}
	}
//	std::cout << all_nodes_histories.size() << "~~~~all~~~~"<< std::endl;
	//	For debug
//	int numid = 0;
//		std::cout << "all cell history output: " << std::endl;
//		for (auto it1 = all_nodes_histories.begin();it1!= all_nodes_histories.end();it1++){
//			std::cout << numid << ": ";
//			for (auto it2 = (*it1).begin(); it2!=(*it1).end(); it2++){
//				std::cout << (*it2)->vertex_id_ << ' ';
//			}
//			numid++;
//			std::cout << std::endl;
//		}
//		std::cout << "history idx: " << std::endl;
//		for (auto it2 = idx_hist.begin();it2 != idx_hist.end(); it2++){
//			std::cout <<(*it2)<< " ";
//		}
//		std::cout << std::endl;

	int first_idx;
	int end_idx;
	int node_1_id = 0;
	int node_2_id = 0;
///Add edges to lifted graph
	for (auto it1 = all_nodes_histories.begin(); it1 != all_nodes_histories.end(); it1++){
		(*it1).front()->lifted_vertices_id_.push_back(node_1_id);
		auto neibour_condition_1 = std::vector<Vertex<SquareCell*>*> ((*it1).begin()+1,(*it1).end());
		auto second_node = *((*it1).begin()+1);
		first_idx = idx_hist[second_node->vertex_id_];
		end_idx = idx_hist[second_node->vertex_id_+1];
		node_2_id = first_idx;
		for (auto it2 = all_nodes_histories.begin()+first_idx; it2 != all_nodes_histories.begin() + end_idx; it2++){

			if ((*it1).front()->vertex_id_ == (*it2).back()->vertex_id_){
				node_2_id++;
				continue;
			}
			if (!(*it1).back()->CheckNeighbour((*it2).back())){
				node_2_id++;
				continue;
			}
			auto neibour_condition_2 = std::vector<Vertex<SquareCell*>*> ((*it2).begin(),(*it2).end()-1);
			if (neibour_condition_1==neibour_condition_2){
				LiftedSquareCell new_lifted_node_1(node_1_id);
				new_lifted_node_1.history = *it1;
				LiftedSquareCell new_lifted_node_2(node_2_id);
				new_lifted_node_2.history = *it2;
				lifted_graph->AddEdge(new_lifted_node_1,new_lifted_node_2, 1);
			}
			node_2_id++;
		}
		node_1_id++;
	}
	return lifted_graph;
}

///Return histories of a vertex using GetHistories_inner_iteration
std::vector<std::vector<Vertex_t<SquareCell*>*>> GraphLifter::GetHistories(Vertex_t<SquareCell*>* actNode, int historyH){
	std::vector<Vertex_t<SquareCell*>*> rawSeq1;
	rawSeq1.push_back(actNode);
	std::vector<std::vector<Vertex_t<SquareCell*>*>> history;
	history = GetHistories_inner_iterative(actNode,historyH+1, 0, rawSeq1, history);
	return history;
}
