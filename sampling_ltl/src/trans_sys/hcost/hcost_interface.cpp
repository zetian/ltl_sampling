/*
 * hcost_interface.cpp

 *
 *  Created on: 18 Nov 2016
 *      Author: zetian
 */
#include "opencv2/opencv.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

#include "trans_sys/hcost/hcost_interface.h"
#include "trans_sys/graph_lifter.h"
#include "map/graph_builder.h"


using namespace cv;
using namespace std;
using namespace srcl;
using namespace Eigen;

//double HCost::get_lifted_transition(int H, std::vector<Vertex<SquareCell>> tile_vertices_Vertex, std::vector<int> rgn_idx_next, std::map<int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL)
double HCost::get_lifted_transition(int H, Matrix<int,Dynamic,4> tile_vertices, std::vector<int> rgn_idx_next, std::map<unsigned int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL)
{
	double cost = 0.0;
	if (!rgn_idx_next.size()){
		rgn_idx_next.clear(); // empty vector
		return cost = 1e9;
	}
	// Convert vector<Vertex> tile_vertices_Vertex to VCell/verts_cd as Eigen Matrix tile_vertices
	// Eigen::MatrixXi tile_vertices;
	// Find equivalent Tile in library
	std::shared_ptr<Tile> this_tile_data = std::make_shared<Tile>(H,tile_vertices); // get_tile_data
	std::shared_ptr<Hlevel> Hlevel = Hlevels.at(H);
	std::map<unsigned int, std::shared_ptr<Tile>> HTiles = Hlevel->Tiles;
	int this_tile_equivalent = 0;

	int m1 = 0;
	for(auto iTiles : HTiles){
		Eigen::Matrix<int,Dynamic,1> this_trav_type = this_tile_data->traversal_type;
		Eigen::Matrix<int,Dynamic,1> test_trav_type = Hlevel->unique_tiles.col(m1);
		if (this_trav_type.isApprox(test_trav_type)){
			this_tile_equivalent = m1;
			break;
		}
		m1++;
	}
	std::shared_ptr<Tile> matched_tile = HTiles.at(this_tile_equivalent);
	std::shared_ptr<Eigen::SparseMatrix<int>> this_connectivity = matched_tile->connectivity;
//	Eigen::MatrixXi this_connectivity_dense = Eigen::MatrixXi(this_connectivity);
	Eigen::MatrixXi this_connectivity_dense = this_connectivity->toDense();
	// Pull out rows from connectivty corresponding to rgn_idx_next
	// Much more of a pain in the end() in C++, Pull each row individually and combine
	// TODO Best way to pull rows and assign in matrix (Dense or Sparse), probably Dense
	//std::shared_ptr<Eigen::SparseMatrix<int>> connect_sparse_block = std::make_shared<Eigen::SparseMatrix<int>>(rgn_idx_next.size(),N_REGION_TOTAL+1);
//	unsigned int size = rgn_idx_next.size();
	Eigen::Matrix<int,Dynamic,Dynamic> connect_block;
	connect_block.resize(rgn_idx_next.size(),N_REGION_TOTAL + 1);
	int m = 0;
	for (auto it:rgn_idx_next){
		connect_block.row(m) = this_connectivity_dense.middleRows(it,1);
		m++;
	}

	Eigen::RowVectorXi sum_row = connect_block.colwise().sum();
	Eigen::RowVectorXi sum_row_binary = (sum_row.array() > 0).cast<int>();

	if (sum_row_binary.sum()){
		cost = 1.0 + std::sqrt((N_REGION_TOTAL + 1 - sum_row_binary.sum())/(N_REGION_TOTAL + 1));
		// Matlab: rgn_idx_next = find(rgn_idx_next) - 1;
		std::vector<int> indices;
		for (int n = 0; n < sum_row_binary.cols(); n++){ // quick equivalent of Matlab find()
			if (sum_row_binary(n)!=0)
				indices.push_back(n);
		}
		rgn_idx_next = indices;
	}else{
		cost = 1.0e9;
		rgn_idx_next.clear();
	}
	return cost;
}

int HCost::zta02rgn_idx(std::vector<double> zta0)
{
	int rgn_idx_init;
	double location = zta0[0];
	double angle = zta0[1];
	REGION_BD REGION_BD; //should be available if hcost_tile_library.h is included
	int region_n = REGION_BD.region_psi_lower.size();
	for (int k = 0; k < region_n; k++){
		if ((location >= REGION_BD.region_w_lower[k]) && (location < REGION_BD.region_w_upper[k]) && (angle >= REGION_BD.region_psi_lower[k]) && (angle < REGION_BD.region_psi_upper[k]))
			rgn_idx_init = k;
	}
	return rgn_idx_init;
}

std::map<unsigned int,std::shared_ptr<Hlevel>> HCost::hcost_preprocessing()
{
	Mat input_image;
		bool use_input_image = false;
		std::shared_ptr<SquareGrid> grid;
		Mat map;


		grid = std::make_shared<SquareGrid>(12,12,1);


		/************************************************************************************/
		/* Below this point, a SquareGrid object should be available for graph construction */
		/************************************************************************************/

		/*** Construct a graph from the square grid ***/
		/*** the second argument determines if move along diagonal is allowed ***/
		auto graph = GraphBuilder::BuildFromSquareGrid(grid,false);

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

		// Create vert_cd, a [rows*cols,4] matrix with vert_cd(i,:) = [xpos,ypos,dx,dy]
		int grid_rows = grid->row_size_;
		MatrixXi verts_cd(grid->row_size_*grid->col_size_,4);
		int m2; int m1; int total_rows = 0;
		int cell_size = grid->cell_size_;
		for (m2 = 0; m2 < grid->row_size_; m2++)
			for (m1 = 0; m1 < grid->col_size_; m1++){
				verts_cd.row(total_rows) << m1*cell_size, m2*cell_size, cell_size, cell_size;
				total_rows++;
			}
		// Verify verts_cd created
		std::cout << "verts_cd" << endl;
		std::cout << verts_cd << endl;

		/********************************************************************
		 * Generate Tile Library
		 * Loop through Hlevels 1 ... H
		 ********************************************************************/
		// map of Hlevels containing all the tiles/tile data
		std::map<unsigned int,std::shared_ptr<Hlevel>> Hlevels;

		Vertex<SquareCell*>* actVert = graph->GetVertexFromID(0);

		for (int HistoryH = 1; HistoryH < MAX_H + 1; HistoryH++){
			std::cout << "***************************************************************" << std::endl;
			std::cout << "***************************************************************" << std::endl;
			std::cout << "At Hlevel H = " << HistoryH << std::endl;
			std::shared_ptr<Hlevel> currHlevel = std::make_shared<Hlevel>(HistoryH);
	//		Hlevels.insert({HistoryH,currHlevel});

			// GetHistory from graphLifter module
			std::vector<std::vector<Vertex<SquareCell*>*>> histories;
			histories = GraphLifter::GetHistories(actVert,HistoryH+1);

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
					tile_vertices.row(ii) << verts_cd.row(histrow(ii));
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

				this_tile_data->connectivity = std::make_shared<Eigen::SparseMatrix<int>>(N_REGION_TOTAL+1,N_REGION_TOTAL+1);
				this_tile_data->connectivity->setFromTriplets(tripletList.begin(), tripletList.end());
			}
		}

		// -------------------- Validate some Connectivity Relations values -------------------
		std::shared_ptr<Hlevel> H1 = Hlevels.at(1);
		std::cout << "H1 unique tiles" << std::endl << H1->unique_tiles << std::endl;
		std::cout << "Made it out alive!!" << std::endl;
		for (auto imap : Hlevels)
			std::cout << imap.second->n_tiles << std::endl;

		return Hlevels;
}


