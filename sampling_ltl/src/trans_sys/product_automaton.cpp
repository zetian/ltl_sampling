/*
 * product_automaton.cpp
 *
 *  Created on: 6 May 2016
 *      Author: zetian
 */
#include <product_automaton.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <bitset>

using namespace srcl;

std::shared_ptr<Graph_t<ProductState>> ProductAutomaton::CreateProductAutomaton(std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph){
	std::shared_ptr<Graph_t<ProductState>> product_automaton_graph = std::make_shared<Graph<ProductState>>();
	std::vector<Vertex<LiftedSquareCell>*> lifted_vertex = lifted_graph->GetGraphVertices();
	std::vector<Vertex<BuchiState>*> buchi_vertex = buchi_graph->GetGraphVertices();
	std::vector<Vertex<LiftedSquareCell>*> neighbour_from_lift;
	int num_buchi_states = buchi_vertex.size();
	std::vector<uint32_t> alphabet_set =buchi_graph->GetVertexFromID(0)->bundled_data_.alphabet_set ;
	std::vector<uint32_t> buchi_transition;
	Vertex<SquareCell*>* last_cell;
	uint32_t cell_region_bit_map;
	uint64_t node_id_1;
	uint64_t node_id_2;

	for (auto it_lifted = lifted_vertex.begin(); it_lifted != lifted_vertex.end(); it_lifted++){
		neighbour_from_lift = (*it_lifted)->GetNeighbours();
		for (auto it_lifted_neighbour = neighbour_from_lift.begin(); it_lifted_neighbour!= neighbour_from_lift.end(); it_lifted_neighbour++){
			for (auto it1 = buchi_vertex.begin();it1 != buchi_vertex.end();it1++){
				for (auto it2 = buchi_vertex.begin();it2 != buchi_vertex.end();it2++){
					buchi_transition = (*it1)->GetBuchiTransition(*it2);
					last_cell = (*it_lifted_neighbour)->bundled_data_.history.back();
					cell_region_bit_map = last_cell->bundled_data_->GetTaskRegionBitMap();
					for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
						if ((*it_buchi_trans)>alphabet_set.size()-1)
								continue;
						if (alphabet_set[(*it_buchi_trans)] == cell_region_bit_map){
							node_id_1 = (*it_lifted)->vertex_id_*num_buchi_states + (*it1)->vertex_id_;
							node_id_2 = (*it_lifted_neighbour)->vertex_id_*num_buchi_states + (*it2)->vertex_id_;
							//===========================================================debug==========================================
//							std::cout <<"~~~~Node " << node_id_1 <<  " and node " <<node_id_2 << " are neighbour" << std::endl;
//							std::cout << "buchi state 1 is " << (*it1)->vertex_id_<< ", buchi state 2 is "<< (*it2)->vertex_id_<< ", and its transition is: ";
//							for (auto it:buchi_transition){
//								std::cout << it << " ";
//							}
//							std::cout << std::endl;
//							std::cout << "Last cell in neighbour: " << last_cell->vertex_id_ << std::endl;
//							std::cout << "Cell region is: " << cell_region.GetRegionName()<< std::endl;
							//===========================================================debug==========================================
							ProductState new_product_state_1(node_id_1);
							new_product_state_1.lifted_vertex_ = *it_lifted;
							new_product_state_1.buchi_vertex_ = *it1;
							ProductState new_product_state_2(node_id_2);
							new_product_state_2.lifted_vertex_ = *it_lifted_neighbour;
							new_product_state_2.buchi_vertex_ = *it2;
							product_automaton_graph->AddEdge(new_product_state_1,new_product_state_2, 1);
							break;
						}
					}
				}
			}
		}
	}
	return product_automaton_graph;
}

uint64_t ProductAutomaton::SetVirtualStart(Vertex<SquareCell *> * start_node_origin, std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph, std::shared_ptr<Graph_t<ProductState>> product_graph){
	std::vector<Vertex<LiftedSquareCell>*> start_node_lifted;
	for (auto it = start_node_origin->lifted_vertices_id_.begin();it != start_node_origin->lifted_vertices_id_.end(); it++){
		start_node_lifted.push_back(lifted_graph->GetVertexFromID((*it)));
//		for(auto i=0; i<lifted_graph->GetVertexFromID((*it))->bundled_data_.history.size();i++){
//		std::cout << lifted_graph->GetVertexFromID((*it))->bundled_data_.history[i]->bundled_data_->GetID()<< "   ";
//		}
//		std::cout << "history" << std::endl;
	}
	auto init_buchi_idx_ = buchi_graph->GetVertexFromID(0)->bundled_data_.init_state_idx_;
	std::vector<Vertex<BuchiState>*> buchi_vertex = buchi_graph->GetGraphVertices();
	int num_buchi_states = buchi_vertex.size();
	std::vector<uint32_t> alphabet_set =buchi_graph->GetVertexFromID(0)->bundled_data_.alphabet_set ;
	std::vector<Vertex<ProductState>*> start_product;
	for(auto it = start_node_lifted.begin(); it != start_node_lifted.end(); it++){
		uint32_t RegionBitMap = (*it)->bundled_data_.history[0]->bundled_data_->GetTaskRegionBitMap();
		for (int i = 1; i < (*it)->bundled_data_.history.size(); i++){
			auto bitmap = (*it)->bundled_data_.history[i]->bundled_data_->GetTaskRegionBitMap();
			RegionBitMap = RegionBitMap|bitmap;
		}
		for (auto ite = buchi_vertex.begin();ite != buchi_vertex.end();ite++){
			std::vector<uint32_t> buchi_transition = buchi_graph->GetVertexFromID(init_buchi_idx_)->GetBuchiTransition(*ite);
			for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
				if (alphabet_set[(*it_buchi_trans)] == RegionBitMap){
					auto product_start_id = (*it)->vertex_id_*num_buchi_states + (*ite)->vertex_id_;
					product_graph->GetVertexFromID(product_start_id)->bundled_data_.buchi_vertex_ = *ite;
					start_product.push_back(product_graph->GetVertexFromID(product_start_id));
				}
			}
		}
	}
	uint64_t virtual_start_id = buchi_graph->GetGraphVertices().size()*lifted_graph->GetGraphVertices().size();
	ProductState virtual_start(virtual_start_id);
	virtual_start.buchi_vertex_ = buchi_graph->GetVertexFromID(init_buchi_idx_);
	for (auto it = start_product.begin(); it!=start_product.end();it++){
		product_graph->AddEdge(virtual_start,(*it)->bundled_data_, 1);
	}
	return virtual_start_id;
}

uint64_t ProductAutomaton::SetVirtualStartIncre(Vertex<SquareCell *> * start_node_origin, std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph, std::shared_ptr<Graph_t<BuchiState>> buchi_graph, std::shared_ptr<Graph<ProductState>> product_graph){
	std::vector<Vertex<LiftedSquareCell>*> start_node_lifted;
	for (auto it = start_node_origin->lifted_vertices_id_.begin();it != start_node_origin->lifted_vertices_id_.end(); it++){
		start_node_lifted.push_back(lifted_graph->GetVertexFromID((*it)));
//		for(auto i=0; i<lifted_graph->GetVertexFromID((*it))->bundled_data_.history.size();i++){
//		std::cout << lifted_graph->GetVertexFromID((*it))->bundled_data_.history[i]->bundled_data_->GetID()<< "   ";
//		}
//		std::cout << "history" << std::endl;
	}

	auto init_buchi_idx_ = buchi_graph->GetVertexFromID(0)->bundled_data_.init_state_idx_;
	std::vector<Vertex<BuchiState>*> buchi_vertex = buchi_graph->GetGraphVertices();
	int num_buchi_states = buchi_vertex.size();
	std::vector<uint32_t> alphabet_set =buchi_graph->GetVertexFromID(0)->bundled_data_.alphabet_set ;
	//std::vector<Vertex<ProductState>*> start_product;
	std::vector<ProductState> start_product;

	for(auto it = start_node_lifted.begin(); it != start_node_lifted.end(); it++){
		uint32_t RegionBitMap = (*it)->bundled_data_.history[0]->bundled_data_->GetTaskRegionBitMap();
		for (int i = 1; i < (*it)->bundled_data_.history.size(); i++){
			auto bitmap = (*it)->bundled_data_.history[i]->bundled_data_->GetTaskRegionBitMap();
			RegionBitMap = RegionBitMap|bitmap;
		}
		for (auto ite = buchi_vertex.begin();ite != buchi_vertex.end();ite++){
			std::vector<uint32_t> buchi_transition = buchi_graph->GetVertexFromID(init_buchi_idx_)->GetBuchiTransition(*ite);
			for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
				if (alphabet_set[(*it_buchi_trans)] == RegionBitMap){
					auto product_start_id = (*it)->vertex_id_*num_buchi_states + (*ite)->vertex_id_;

					ProductState product_start(product_start_id);
					product_start.buchi_vertex_ = *ite;
					product_start.lifted_vertex_ = *it;
					start_product.push_back(product_start);
				}
			}
		}
	}
	uint64_t virtual_start_id = buchi_graph->GetGraphVertices().size()*lifted_graph->GetGraphVertices().size();

	ProductState virtual_start(virtual_start_id);

	virtual_start.buchi_vertex_ = buchi_graph->GetVertexFromID(init_buchi_idx_);
	for (auto it = start_product.begin(); it!=start_product.end();it++){
		product_graph->AddEdge(virtual_start,*it, 1);
	}

	return virtual_start_id;
}


std::vector<std::tuple<ProductState, double>> GetProductCBTANeighbour::operator()(ProductState cell){
	std::vector<std::tuple<ProductState, double>> adjacent_cells;
	std::vector<uint32_t> alphabet_set = buchi_graph_->GetVertexFromID(0)->bundled_data_.alphabet_set;
	std::vector<uint32_t> buchi_transition;
	Vertex<SquareCell*>* last_cell;
	uint32_t cell_region_bit_map;
	uint64_t node_id_new;
	uint64_t edge_cost;
	Vertex<LiftedSquareCell>* current_lifted_vertex = cell.lifted_vertex_;
	Vertex<BuchiState>* current_buchi_vertex = cell.buchi_vertex_;
	std::vector<Vertex<LiftedSquareCell>*> neighbour_from_lift = current_lifted_vertex->GetNeighbours();
	std::vector<Vertex<BuchiState>*> buchi_vertex = buchi_graph_->GetGraphVertices();
	int num_buchi_states = buchi_vertex.size();

	for (auto it_lifted_neighbour = neighbour_from_lift.begin(); it_lifted_neighbour!= neighbour_from_lift.end(); it_lifted_neighbour++){
		for (auto it_buchi = buchi_vertex.begin();it_buchi != buchi_vertex.end();it_buchi++){
			buchi_transition = current_buchi_vertex->GetBuchiTransition(*it_buchi);
			last_cell = (*it_lifted_neighbour)->bundled_data_.history.back();
			cell_region_bit_map = last_cell->bundled_data_->GetTaskRegionBitMap();
			for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
				if ((*it_buchi_trans)>alphabet_set.size()-1)
					continue;
				if (alphabet_set[(*it_buchi_trans)] == cell_region_bit_map){
					node_id_new = (*it_lifted_neighbour)->vertex_id_*num_buchi_states + (*it_buchi)->vertex_id_;
					ProductState new_product_state(node_id_new);
					new_product_state.lifted_vertex_ = *it_lifted_neighbour;
					new_product_state.buchi_vertex_ = *it_buchi;
					edge_cost = 1;
					adjacent_cells.emplace_back(new_product_state, edge_cost);
					break;
				}
			}
		}
	}
	return adjacent_cells;
}
