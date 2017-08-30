/*
 * calc_hcost_main.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: ben
 */
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <algorithm>
#include <functional>
#include <time.h>
#include <stdio.h>
#include <ctime>
#include <tuple>
#include <memory>

// opencv
#include "opencv2/opencv.hpp"

// Eigen header
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/QR>

// user
#include "map/sgrid_builder.h"
#include "graph/astar.h"
#include "graph/graph.h"
#include "map/graph_builder.h"
#include "map/map_utils.h"
#include "vis/graph_vis.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/hcost_tile_library.h"
#include "trans_sys/product_automaton.h"
#include "trans_sys/buchi_automaton.h"



using namespace cv;
using namespace std;
using namespace srcl;
using namespace Eigen;

// Define Tile data matrices

int main(int argc, char** argv )
{
	Mat input_image;
	bool use_input_image = false;
	std::shared_ptr<SquareGrid> grid_cbta;
	Mat map;

	/*** check if user specifies an image ***/
	if ( argc == 2 )
	{
		input_image = imread( argv[1], IMREAD_GRAYSCALE );

		if (!input_image.data)
		{
			printf("No image data \n");
			return -1;
		}
		/*** create a square grid map from input image ***/
		else
		{
			/*** BuildSquareGridMap() returns both the square grid data structure    ***/
			/***  and the post-processed image, this image is usually not the same   ***/
			/***  with the original input image (after binarizing, padding). The     ***/
			/***  square grid or the graph can be visualized over this image without ***/
			/***  mis-placement. ***/
//			std::tuple<std::shared_ptr<SquareGrid>, Mat> sg_map;
//
//			sg_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
//			grid = std::get<0>(sg_map);
//			map = std::get<1>(sg_map);

			/*** BuildSquareGrid() only returns the square grid data structure ***/
			//grid = SGridBuilder::BuildSquareGrid(input_image, 32);

			use_input_image = true;
		}
	}
	/*** otherwise, create a square grid map manually ***/
	else{
		// create a empty grid
		grid_cbta = std::make_shared<SquareGrid>(6,6,1);

		// set occupancy for cells

	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	auto graph_cbta = GraphBuilder::BuildFromSquareGrid(grid_cbta,false);
	GraphLifter graph_lifter;

/************************************************************************************/
/* Everything below here is for developing/testing CBTA code */
/************************************************************************************/

	/**************** GENERAL PARAMETERS FOR OVERALL HCOST LIBRARY ********************
	* Numbers of regions in distance, orientation, and velocity
	* Numbering scheme:
	* N_REGION_VEL*N_REGION_PSI*k_vel + N_REGION_PSI*k_psi + k_w
	* Region numbering must start at 0
	*/
	int N_REGION_W = 25;
	int N_REGION_PSI = 25;
	int N_REGION_SPD = 1;
	int N_REGION_TOTAL = N_REGION_W*N_REGION_PSI*N_REGION_SPD - 1;

	// Extent of these regions in distance, orientation, velocity
	double SIZE_REGION_W = 1.0/N_REGION_W;
	double SIZE_REGION_PSI = PI/N_REGION_PSI;
	double SIZE_REGION_SPD = (SPD_MAX - SPD_MIN)/N_REGION_SPD;

	//int r_min = 3;
	REGION_BD REGION_BD;

	int k_w, k_psi, k_vel;
	for (int m1 = 0; m1 < N_REGION_TOTAL + 1;m1++){
		k_w = (m1%(N_REGION_SPD*N_REGION_PSI))%N_REGION_PSI;
		k_psi = floor((m1%(N_REGION_W*N_REGION_PSI))/N_REGION_PSI);
		k_vel = floor(m1/N_REGION_PSI/N_REGION_W);

		REGION_BD.region_w_lower.push_back(k_w*SIZE_REGION_W);
		REGION_BD.region_w_upper.push_back((k_w+1)*SIZE_REGION_W);
		REGION_BD.region_psi_lower.push_back(-PI/2 + k_psi*SIZE_REGION_PSI);
		REGION_BD.region_psi_upper.push_back(-PI/2 + (k_psi+1)*SIZE_REGION_PSI);
		REGION_BD.region_vel_lower.push_back(SPD_MIN + k_vel*SIZE_REGION_SPD);
		REGION_BD.region_vel_upper.push_back(SPD_MIN + (k_vel+1)*SIZE_REGION_SPD);
	}

	std::cout << "REGION_BD first column" << std::endl;
	std::cout << REGION_BD.region_w_lower[0] << std::endl;
	std::cout << REGION_BD.region_w_upper[0] << std::endl; // output differs MATLAB
	std::cout << REGION_BD.region_psi_lower[0] << std::endl;
	std::cout << REGION_BD.region_psi_upper[0] << std::endl;// output differs MATLAB
	std::cout << REGION_BD.region_vel_lower[0] << std::endl;
	std::cout << REGION_BD.region_vel_upper[0] << std::endl;

//	// Create vert_cd, a [rows*cols,4] matrix with vert_cd(i,:) = [xpos,ypos,dx,dy]
//	int grid_rows_cbta = grid_cbta->row_size_;
//	MatrixXi verts_cd_cbta(grid_cbta->row_size_*grid_cbta->col_size_,4);
//	int total_rows_cbta = 0;
//	int cell_size_cbta = grid_cbta->cell_size_;
//	for (int m2 = 0; m2 < grid_cbta->row_size_; m2++)
//		for (int m1 = 0; m1 < grid_cbta->col_size_; m1++){
//			verts_cd_cbta.row(cell_size_cbta) << m1*cell_size_cbta, m2*cell_size_cbta, cell_size_cbta, cell_size_cbta;
//			cell_size_cbta++;
//		}
//	// Verify verts_cd created
//	std::cout << "verts_cd_cbta" << endl;
//	std::cout << verts_cd_cbta << endl;

	// Create vert_cd, a [rows*cols,4] matrix with vert_cd(i,:) = [xpos,ypos,dx,dy]
//	int grid_rows = grid_cbta->row_size_;
	MatrixXi verts_cd_cbta(grid_cbta->row_size_*grid_cbta->col_size_,4);
	int total_rows_cbta = 0;
	int cell_size_cbta = grid_cbta->cell_size_;
	for (int m2 = 0; m2 < grid_cbta->row_size_; m2++)
		for (int m1 = 0; m1 < grid_cbta->col_size_; m1++){
			verts_cd_cbta.row(total_rows_cbta) << m1*cell_size_cbta, m2*cell_size_cbta, cell_size_cbta, cell_size_cbta;
			total_rows_cbta++;
		}
	// Verify verts_cd created
	std::cout << "verts_cd_cbta" << endl;
	std::cout << verts_cd_cbta << endl;



	/********************************************************************
	 * Generate Tile Library
	 * Loop through Hlevels 1 ... H
	 ********************************************************************/
	// map of Hlevels containing all the tiles/tile data
	std::map<unsigned int,std::shared_ptr<Hlevel>> Hlevels;

	Vertex<SquareCell*>* actVert = graph_cbta->GetVertexFromID(0);

	for (int HistoryH = 1; HistoryH < MAX_H + 1; HistoryH++){
		std::cout << "***************************************************************" << std::endl;
		std::cout << "***************************************************************" << std::endl;
		std::cout << "At Hlevel H = " << HistoryH << std::endl;
		std::shared_ptr<Hlevel> currHlevel = std::make_shared<Hlevel>(HistoryH);
//		Hlevels.insert({HistoryH,currHlevel});

		// GetHistory from graphLifter module
		std::vector<std::vector<Vertex<SquareCell*>*>> histories;
		histories = graph_lifter.GetHistories(actVert,HistoryH+1);

		// Convert GetHistory result into matrix form for Tile computations
		MatrixXi all_hist(histories.size(), histories[0].size());
		vector< vector<Vertex<SquareCell*>*>>::reverse_iterator row;
		vector< Vertex<SquareCell*>*>::iterator col;
		int mrow = 0;
		int mcol = 0;
		for (row = histories.rbegin(); row != histories.rend(); ++row){ // history returned backward
			mcol = 0;                                                   // from MATLAB code
			for (col = (*row).begin(); col!=(*row).end(); ++col){
				all_hist(mrow,mcol) = (*col)->vertex_id_;
				mcol++;
			}
			mrow++;
		}
		// Find unique traversals
		// Uniqueness of Tile determined when passed to current Hlevel
		for (int histrow_no = 0; histrow_no < all_hist.rows(); histrow_no++){
			RowVectorXi histrow = all_hist.row(histrow_no);
			MatrixXi tile_vertices(histrow.size(),4);
			for (int ii = 0; ii < histrow.size(); ii++){
				tile_vertices.row(ii) << verts_cd_cbta.row(histrow(ii));
			}
			// Construct Tile and set data
			std::shared_ptr<Tile> this_tile = std::make_shared<Tile>(HistoryH,tile_vertices);

			currHlevel->add_tile(this_tile); // Hlevel stores Tiles (pointer) and determines uniqueness
		}

		std::cout << currHlevel->Tiles.size() << " tiles in current Hlevel" << std::endl;

		Hlevels.insert({HistoryH,currHlevel});
	}
	/*****************************************************************************
	 *  CONNECTIVITY RELATIONS
	 *  This section iterates through the Hlevels and through the Tiles within the
	 *  Hlevels. CBTA results for each tiles are computed during this section	 *
	 */
	std::cout << " ----------**********---------**********-------------***********" << std::endl;
	std::cout << "START CONNECTIVITY RELATIONS" << std::endl;
	std::cout << " ----------**********---------**********-------------***********" << std::endl;
	// for each Hlevel
	//     for each Tile
	//         currTile.addTileBlock()
	for (int HistoryH = 1; HistoryH < MAX_H + 1; HistoryH++){
		std::shared_ptr<Hlevel> currHlevel = Hlevels.at(HistoryH);
		std::cout << "History Level for loop at H = " << HistoryH << std::endl;
		for (auto it = currHlevel->Tiles.begin(); it != currHlevel->Tiles.end(); ++it){
			MatrixXi this_tile_edge_list = MatrixXi::Zero(0,3);
			int n_this_tile_edges = 0;

			std::shared_ptr<Tile> this_tile_data = it->second;
			std::cout << "--------- Tile data operating on -----------" << std::endl;
			std::cout << "channel_data" << std::endl << this_tile_data->channel_data << std::endl;
			std::cout << "cell_vertices" << std::endl << this_tile_data->cell_vertices << std::endl;
			std::cout << "traversal_type" << std::endl << this_tile_data->traversal_type << std::endl;
			std::cout << "cell_xform" << std::endl << this_tile_data->cell_xform << std::endl;
			std::cout << "traversal_faces" << std::endl << this_tile_data->traversal_faces << std::endl;
			std::cout << "cell_edge" << std::endl << this_tile_data->cell_edge << std::endl;
			std::cout << " -------- End of current Tile data attributes----------" << std::endl;
			std::shared_ptr<TileBlock> this_tile_block = std::make_shared<TileBlock>(REGION_BD,this_tile_data,HistoryH);
			this_tile_data->addTileBlock(REGION_BD,this_tile_block,HistoryH);
			this_tile_block->cbta(); // Activte CBTA computations, necessary cbta_results
			                        // stored in this_tile_block's public members

			// ------- Find grid regions at end of second cell from where traversal
			// throughout is possible (everywhere -> somewhere)
			// These are the Q sets described in the paper(s), we are
			// identifying a "discretized" approximation to it..

			// Called "cbta_target_2" and "cbta_target_1" (target sets for first/second cells)
			Matrix<double,2,N_CBTA_W> cbta_target_1_alfa;
			RowVectorXd cbta_target_1_w;
			Matrix<double,2,N_CBTA_W> cbta_target_2_alfa;
			RowVectorXd cbta_target_2_w;
			if (HistoryH == 1){                                               // The Q set is "complete" (all regions are in it).
				cbta_target_2_alfa.row(0) =  PI/2.0*MatrixXd::Ones(1,N_CBTA_W);
				cbta_target_2_alfa.row(1) = -PI/2.0*MatrixXd::Ones(1,N_CBTA_W);
				cbta_target_2_w = RowVectorXd::LinSpaced(N_CBTA_W,0,1);

			}else{
				cbta_target_2_alfa = this_tile_block->alfa.middleRows(2,2); // cbta_results.alfa(3:4,:)
				cbta_target_2_w	   = this_tile_block->w.middleRows(1,1);	// cbta_results.w(2,:)
			}
			cbta_target_1_alfa = this_tile_block->alfa.topRows(2);          // cbta_results.alfa(1:2,:)
			cbta_target_1_w    = this_tile_block->w.topRows(1);				// cbta_results.w(1,:)
			std::cout << "pulling out cbta_target 1,2 from cbta_results" << std::endl;

			RowVectorXi region_target_2 = RowVectorXi::Ones(N_REGION_TOTAL + 1); // Intersection with CBTA computed target
			for (int m1 = 0; m1 < N_REGION_TOTAL; m1++){

				double region_w_lower   = REGION_BD.region_w_lower.at(m1);
				double region_w_upper   = REGION_BD.region_w_upper.at(m1);
				double region_psi_lower = REGION_BD.region_psi_lower.at(m1);
				double region_psi_upper = REGION_BD.region_psi_upper.at(m1);

				if ((region_w_lower > cbta_target_2_w.tail(1).value()) || (region_w_upper < cbta_target_2_w(0)))
					continue;

				int region_w_lower_idx = TileBlock::find_sample(cbta_target_2_w,region_w_lower);
				int region_w_upper_idx = TileBlock::find_sample(cbta_target_2_w,region_w_upper);

				RowVectorXd cbta_target_2_alfa1_seg = cbta_target_2_alfa.row(0).segment(region_w_lower_idx,region_w_upper_idx - region_w_lower_idx + 1);
				RowVectorXd cbta_target_2_alfa2_seg = cbta_target_2_alfa.row(1).segment(region_w_lower_idx,region_w_upper_idx - region_w_lower_idx + 1);

				if ( ((cbta_target_2_alfa1_seg.array() < region_psi_lower) + // '+' replaces bitwise OR '|' from MATLAB code
					  (cbta_target_2_alfa2_seg.array() > region_psi_upper)).all() )
					region_target_2(m1) = 0;
			}
//			std::cout << "Connectivity Relations: finished assigning region_target_2" << std::endl;

			RowVectorXi region_target_1 = RowVectorXi::Zero(N_REGION_TOTAL + 1); //Inclusion under CBTA computed target
			for (int m1 = 0; m1 < N_REGION_TOTAL; m1++){
				double region_w_lower   = REGION_BD.region_w_lower.at(m1);
				double region_w_upper   = REGION_BD.region_w_upper.at(m1);
				double region_psi_lower = REGION_BD.region_psi_lower.at(m1);
				double region_psi_upper = REGION_BD.region_psi_upper.at(m1);

				if ((region_w_lower < cbta_target_1_w(0)) || (region_w_upper > cbta_target_1_w.tail(1).value()))
					continue;

				int region_w_lower_idx = TileBlock::find_sample(cbta_target_1_w,region_w_lower);
				int region_w_upper_idx = TileBlock::find_sample(cbta_target_1_w,region_w_upper);

				RowVectorXd cbta_target_1_alfa1_seg = cbta_target_1_alfa.row(0).segment(region_w_lower_idx,region_w_upper_idx - region_w_lower_idx + 1);
				RowVectorXd cbta_target_1_alfa2_seg = cbta_target_1_alfa.row(1).segment(region_w_lower_idx,region_w_upper_idx - region_w_lower_idx + 1);

				ArrayXXd alfa1_seg; ArrayXXd alfa2_seg;
				alfa1_seg.resize(1,cbta_target_1_alfa1_seg.cols());
				alfa1_seg = cbta_target_1_alfa1_seg;
				alfa2_seg.resize(1,cbta_target_1_alfa2_seg.cols());
				alfa2_seg = cbta_target_1_alfa2_seg;
				// 'A.cwiseProduct(B)' replaces bitwise 'A&B' from MATLAB code
//				MatrixXi prodAB = ((alfa1_seg >= region_psi_upper).cast<int>()).cwiseProduct((alfa2_seg <= region_psi_lower).cast<int>());

				MatrixXi prodAB = ((alfa1_seg - region_psi_upper >= TOL).cast<int>()).cwiseProduct((alfa2_seg - region_psi_lower <= TOL).cast<int>()); // address floating pt equality
				if ( prodAB.all() )
					region_target_1(m1) = 1;
			}
//			std::cout << "Connectivity Relations: finished assigning region_target_1" << std::endl;
			// ------- Find grid regions at the end of first cell from where
			// traversal is possible (everywhere -> somewhere), call this Q_0
			// ------- For each of these, figure out reachable grid regions
			// ------- Intersect these grid regions with Q, and get the "to"
			// regions in the connectivity graph
			// this_tile_edge_list.resize((int)region_neighbors.cols(),3);
			for (int m1 = 0; m1 < N_REGION_TOTAL + 1; m1++){
				// ---- If this region not in Q_0, then do nothing
				if (!region_target_1(m1)) continue;

				RowVectorXi region_neighbors;
				// call cbra (Curvature Bounded Reachablility Analysis) to find reachable region neighbors
				this_tile_block->cbra(m1,REGION_BD,r_min,N_REGION_TOTAL,region_target_2,region_neighbors);
				// ---- Intersect reachable region with Q, record edge
				this_tile_edge_list.conservativeResize(this_tile_edge_list.rows()+region_neighbors.cols(),NoChange); // Add rows
				this_tile_edge_list.block(n_this_tile_edges,0,region_neighbors.cols(),1) = m1*VectorXi::Ones(region_neighbors.cols());
				this_tile_edge_list.block(n_this_tile_edges,1,region_neighbors.cols(),1) = region_neighbors.transpose();
				this_tile_edge_list.block(n_this_tile_edges,2,region_neighbors.cols(),1) = VectorXi::Ones(region_neighbors.cols());
				n_this_tile_edges += region_neighbors.cols();
			}
//			std::cout << "Connectivity Relations: past region_neighbors assignment" << std::endl;
			std::cout << "Number of NonZero Entries in Connectivity: " << n_this_tile_edges << std::endl;
			// ---- Place this_tile_edge_list stuff into Sparse Matrix Connectivity
			typedef Eigen::Triplet<double> T;
			std::vector<T> tripletList;
			tripletList.reserve(this_tile_edge_list.rows());
			for (int k = 0; k < this_tile_edge_list.rows(); k++){
				tripletList.push_back(T(this_tile_edge_list(k,0),this_tile_edge_list(k,1),this_tile_edge_list(k,2))); // T(i,j,v_ij)
			}
			//Eigen::SparseMatrix<int> connectivity(N_REGION_TOTAL+1,N_REGION_TOTAL+1);
			//this_tile_data->connectivity(N_REGION_TOTAL+1,N_REGION_TOTAL+1);
			this_tile_data->connectivity = std::make_shared<Eigen::SparseMatrix<int>>(N_REGION_TOTAL+1,N_REGION_TOTAL+1);
			//this_tile_data->connectivity->resize(N_REGION_TOTAL+1,N_REGION_TOTAL+1);
			this_tile_data->connectivity->setFromTriplets(tripletList.begin(), tripletList.end());
		}
	}

	// -------------------- Validate some Connectivity Relations values -------------------
	std::shared_ptr<Hlevel> H1 = Hlevels.at(1);
	std::cout << "H1 unique tiles" << std::endl << H1->unique_tiles << std::endl;
	std::cout << "Made it out alive!!" << std::endl;
	for (auto imap : Hlevels)
		std::cout << imap.second->n_tiles << std::endl;


	/*** 1. Create a empty square grid ***/
		int row_num = 12;
		int col_num = 12;

		std::vector<std::vector<int>> Vcell;

//		for (auto it1 = 0;it1 < row_num; it1++){
//			for (auto it2 = 0;it2 < col_num; it2++){
//				int row[] = {it2,it1,1,1};
//				std::vector<int> Vcell_row(row, row + sizeof(row) / sizeof(int) );
//				Vcell.push_back(Vcell_row);
//			}
//		}
//
//		for (auto it1 = 0;it1 < col_num*col_num; it1++){
//			for (auto it2 = 0;it2 < 4; it2++){
//				std::cout << Vcell[it1][it2]<< " ";
//			}
//			std::cout << std::endl;
//		}


		int tile_size = 50;


		std::shared_ptr<SquareGrid> grid = MapUtils::CreateSquareGrid(row_num,col_num,tile_size);



//
		// Create vert_cd, a [rows*cols,4] matrix with vert_cd(i,:) = [xpos,ypos,dx,dy]
		int grid_rows = grid->row_size_;
		MatrixXi verts_cd(grid->row_size_*grid->col_size_,4);
		int total_rows = 0;
		int cell_size = grid->cell_size_;
		for (int m2 = 0; m2 < grid->row_size_; m2++)
			for (int m1 = 0; m1 < grid->col_size_; m1++){
				verts_cd.row(total_rows) << m1*cell_size, m2*cell_size, cell_size, cell_size;
				total_rows++;
			}
		std::cout << "~~~~~~~" << endl;
		std::cout << verts_cd << endl;






		// assign properties of square cells
		for(auto it = 0; it < (row_num*col_num) ; it++){
			grid->cells_[it]->AssignRegionLabel(0);
		}

		for(auto i = 120; i < 129; i++){
			grid->cells_[i]->RemoveRegionLabel(0);
			grid->cells_[i]->AssignRegionLabel(1);
			grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);
		}


		grid->cells_[110]->AssignRegionLabel(2);
		grid->SetCellOccupancy(110, OccupancyType::INTERESTED);

		grid->cells_[115]->AssignRegionLabel(3);
		grid->SetCellOccupancy(115, OccupancyType::INTERESTED);

//		grid->cells_[66]->AssignRegionLabel(4);
//		grid->SetCellOccupancy(66, OccupancyType::INTERESTED);
//
//		grid->cells_[99]->AssignRegionLabel(5);
//		grid->SetCellOccupancy(99, OccupancyType::INTERESTED);

		/*** 2. Construct a graph from the square grid ***/
		std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid_IgnoreObstacle(grid,false);

		/*** 3. Construct a graph from the buchi automata ***/
	//	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p3) && (<> p4) && ((<> p2) || (<> p5))";
//		std::string ltl_formula = "([] p0) && ([] !p1) && (<> p3) && (<> p4) && (<> p2) && (<> p5)";
		std::string ltl_formula = "[] p0 && [] !p1 && <> (p2 && <> p3)";
	//	std::string ltl_formula = "([] p0) && ([] ! p1) && ( <> p4 ||<> p5 || <> p2 )";
		std::vector<std::string> buchi_regions;
		buchi_regions.push_back("p0");
		buchi_regions.push_back("p1");
		buchi_regions.push_back("p2");
	//	buchi_regions.push_back("p5");
		buchi_regions.push_back("p3");
//		buchi_regions.push_back("p5");
//		buchi_regions.push_back("p4");

		std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions);

		/*** 4. Construct a lifted graph ***/
		int HistoryH = 4;
		std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);

		/*** 5. Construct a product graph ***/
		clock_t     total_time;
		clock_t		product_time;
		total_time = clock();
		product_time = clock();
	//	std::shared_ptr<Graph_t<ProductState>> product_graph = ProductAutomaton::CreateProductAutomaton(lifted_graph,buchi_graph);
		std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();
		product_time = clock() - product_time;
	//	std::cout << "Product graph time in " << double(product_time)/CLOCKS_PER_SEC << " s." << std::endl;

		/*** 6. Search in the product graph ***/
		// Set start node in original graph
		Vertex<SquareCell *> * start_node_origin = graph->GetVertexFromID(132);

		// Convert from original node to product node
		uint64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, lifted_graph, buchi_graph, product_graph_new);
	//	uint64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, lifted_graph, buchi_graph, product_graph);

	//	auto virtual_start = product_graph->GetVertexFromID(virtual_start_id);
		// Search in product graph
		std::vector<uint32_t>  buchi_acc = buchi_graph->GetVertexFromID(0)->bundled_data_.acc_state_idx;

		GetProductCBTANeighbour get_product_CBTA_neighbour(lifted_graph, buchi_graph, Hlevels, grid);


		std::vector<ProductState> path = AStar::ProductIncSearch(product_graph_new, virtual_start_id,buchi_acc, GetNeighbourBDSFunc_t<ProductState>(GetProductCBTANeighbour(lifted_graph, buchi_graph, Hlevels, grid)));
	//	std::vector<Vertex<ProductState>*> path = AStar::ProductSearch(product_graph, virtual_start,buchi_acc);

		total_time = clock() - total_time;
		std::cout << "Total time in " << double(total_time)/CLOCKS_PER_SEC << " s." << std::endl;
		// For debug, check path detail
		//	std::cout << "location: "<<std::endl;
		//	for (auto it = path.begin()+1; it!=path.end();it++){
		//		std::cout <<"Product id: " << (*it)->bundled_data_.GetID() <<", history:";
		//			for (auto ite = (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.begin();ite != (*it)->bundled_data_.lifted_vertex_->bundled_data_.history.end(); ite++){
		//				std::cout << " "<<(*ite)->bundled_data_->GetID();
		//			}
		//		std::cout << ", Buchi state: " <<  (*it)->bundled_data_.buchi_vertex_->bundled_data_.GetID();
		//		std::cout << std::endl;
		//	}

		// Map path in the product graph back to the square grid graph

		std::vector<Vertex<SquareCell*>*> path_origin;
		for (auto it = path.begin()+1; it != path.end(); it++){
			path_origin.push_back((*it).lifted_vertex_->bundled_data_.history.front());
		}
		path_origin.insert(path_origin.end(),path.back().lifted_vertex_->bundled_data_.history.begin()+1,path.back().lifted_vertex_->bundled_data_.history.end());

		/*** 7. Visualize the map and graph ***/
		// Image Layouts: square grid -> graph -> path
		GraphVis vis;
		Mat vis_img;
		vis.VisSquareGrid(*grid, vis_img);
		vis.VisSquareGridGraph(*graph, vis_img, vis_img, true);
		vis.VisSquareGridPath(path_origin, vis_img, vis_img);

		// display visualization result
		namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
		imshow("Processed Image", vis_img);

		waitKey(0);





	return 0;
}



