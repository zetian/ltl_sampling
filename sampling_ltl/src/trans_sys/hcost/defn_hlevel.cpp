/*
 * defn_Hlevel.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: bscooper
 */

// Eigen header
#include <eigen3/Eigen/Core>
#include "trans_sys/hcost/hcost_tile_library.h"

using namespace Eigen;
using namespace srcl;

/* ------- Hlevel constructors and destructors -------- */
Hlevel::Hlevel(unsigned int Hin){
	H = Hin;
	n_tiles = 0;
	unique_tiles = MatrixXi::Zero(H,1);

}

Hlevel::~Hlevel(){

}

void Hlevel::get_tile_data(){
	// get_history of vector<vector<>> and convert to Eigen matrix
	// 'verts_cd' built from hcost_tile_library
	// 'graph_cd from 'get_adjacency_matrix_4conn(verts_cd)
	// vector<vector<unsigned long int> > start_vertex_histories;
	// get_history(start_vertex_name, lift_H, start_vertex_histories);

}

void Hlevel::add_tile(std::shared_ptr<Tile> newTile){
	// If tile is unique, add to Tiles, increment n_tiles, and update unique_tiles
	bool is_new_tile = true;
	if (n_tiles == 0){
		unique_tiles = newTile->traversal_type;
	}else{
		// need to look through all unique_tiles cols to find match for newTile traversal_type
		for (int tile_col = 0; tile_col < unique_tiles.cols(); tile_col++){
			if (newTile->traversal_type.isApprox(unique_tiles.col(tile_col))){
				is_new_tile = false;
				break;
			}
		}
	}
	if (is_new_tile){
		std::cout << "is new tile" << std::endl;
		unique_tiles.conservativeResize(NoChange, unique_tiles.cols()+1);
		unique_tiles.col(unique_tiles.cols()-1) = newTile->traversal_type;
		Tiles.insert({n_tiles,newTile});
		n_tiles++;
	}
}

