/*
 * test_tile_data.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: bscooper
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
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/QR>

// user
#include "map/sgrid_builder.h"
#include "graph/graph.h"
#include "map/graph_builder.h"
#include "vis/graph_vis.h"
#include "trans_sys/graph_lifter.h"
#include "trans_sys/hcost/hcost_tile_library.h"
#include "trans_sys/product_automaton.h"

using namespace cv;
using namespace std;
using namespace srcl;
using namespace Eigen;

// Define Tile data matrices

int main(int argc, char** argv )
{
	Mat input_image;
	bool use_input_image = false;
	std::shared_ptr<SquareGrid> grid;
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
			Map_t<SquareGrid> sg_map;

			sg_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
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
		grid = std::make_shared<SquareGrid>(5,5,50);

		// set occupancy for cells

	}

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
//	std::shared_ptr<Graph<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(grid,false);
	auto graph = GraphBuilder::BuildFromSquareGrid(grid,false);
//	std::vector<Vertex<SquareCell>*> all_verts_origin = graph->GetGraphVertices();

	unsigned int HistoryH = 5;
//	Graph<LiftedSquareCell>* LiftedGraph;
	auto LiftedGraph = GraphLifter::CreateLiftedGraph(HistoryH, graph);

	// Get a history
	Vertex<SquareCell*>* actVert = graph->GetVertexFromID(0);
	std::vector<std::vector<Vertex<SquareCell*>*>> histories;
	histories = GraphLifter::GetHistories(actVert,HistoryH+1);


/************************************************************************************/
/* Everything below here is for developing/testing CBTA code */
/************************************************************************************/

	/**************** GENERAL PARAMETERS FOR OVERALL HCOST LIBRARY ********************
	* Numbers of regions in distance, orientation, and velocity
	* Numbering scheme:
	* N_REGION_VEL*N_REGION_PSI*k_vel + N_REGION_PSI*k_psi + k_w
	* Region numbering must start at 0
	*/
	int N_REGION_W = 75;
	int N_REGION_PSI = 75;
	int N_REGION_SPD = 1;
	int N_REGION_TOTAL = N_REGION_W*N_REGION_PSI*N_REGION_SPD;

	// Extent of these regions in distance, orientation, velocity
	double SIZE_REGION_W = 1.0/N_REGION_W;
	double SIZE_REGION_PSI = PI/N_REGION_PSI;
	double SIZE_REGION_SPD = (SPD_MAX - SPD_MIN)/N_REGION_SPD;

	int r_min = 3;

	// Region boundaries
//	struct REGION_BD{
//		std::vector<double> region_w_lower;
//		std::vector<double> region_w_upper;
//		std::vector<double> region_psi_lower;
//		std::vector<double> region_psi_upper;
//		std::vector<double> region_vel_lower;
//		std::vector<double> region_vel_upper;
//	}REGION_BD;
	REGION_BD REGION_BD;

	int k_w, k_psi, k_vel;
	for (int m1 = 0; m1 < N_REGION_TOTAL;m1++){
		k_w = (m1%(N_REGION_SPD*N_REGION_PSI))%N_REGION_PSI;
		k_psi = floor((m1%(N_REGION_W*N_REGION_PSI))%N_REGION_PSI);
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

	// map of Hlevels containing all the tiles/tile data
	std::map<unsigned int,Hlevel> Hlevels;
	Hlevel currHlevel = Hlevel(HistoryH);
	Hlevels.insert({HistoryH,currHlevel});

	// Construct Tile and set data
	MatrixXi all_hist(histories.size(), histories[0].size());

	// Convert GetHistory result into matrix form for Tile computations
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
	// Verify history conversion
	std::cout << "all_hist" << std::endl;
	std::cout << all_hist << std::endl;

	for (int histrow_no = 0; histrow_no < all_hist.rows(); histrow_no++){

		VectorXi histrow = all_hist.row(histrow_no);
		std::cout << "--------------------------------------------------" << std::endl;
		std::cout << "row " << histrow_no << ": " << histrow.transpose() << std::endl;
		MatrixXi tile_vertices(histrow.size(),4);
//		std::cout << "verts_cd.row(histrow(0))" << std::endl;
//		std::cout << verts_cd.row(histrow(0)) << std::endl;
		for (int ii = 0; ii < histrow.size(); ii++){
			tile_vertices.row(ii) << verts_cd.row(histrow(ii));
		}

		std::shared_ptr<Tile> this_tile = std::make_shared<Tile>(HistoryH,tile_vertices);
//		std::cout << "tile_vertices" << std::endl;
//		std::cout << tile_vertices << std::endl;
		std::cout << this_tile->cell_vertices << std::endl;

		currHlevel.add_tile(this_tile); // pass by reference?
	}

	std::cout << currHlevel.Tiles.size() << " tiles in current Hlevel" << std::endl;

	for(auto it = currHlevel.Tiles.begin();it != currHlevel.Tiles.end();++it){
		std::cout << "------------------------------------------------------------" << std::endl;
		std::cout << "Tile: " << it->first << std::endl;
		std::shared_ptr<Tile> curr_tile = it->second;
		std::cout << "Traversal_type: " << curr_tile->traversal_type.transpose() << std::endl;
		std::cout << curr_tile->cell_vertices << std::endl;
		std::cout << "channel_data = chan_mid" << std::endl;
		std::cout << curr_tile->channel_data << std::endl;
	}

	/*****************************************************************************
	 *  CONNECTIVITY RELATIONS
	 *  This section iterates through the Hlevels and through the Tiles within the
	 *  Hlevels. CBTA results for each tiles are computed during this section	 *
	 */

	// for each Hlevel
	//     for each Tile
	//         currTile.addTileBlock()
	std::cout << "Hlevel = " << currHlevel.H << " with " << currHlevel.n_tiles << " tiles" << std::endl;
	std::shared_ptr<Tile> currentTile = currHlevel.Tiles.at(0);
	std::cout << "currentTile->cell_xform" << std::endl;
	std::cout << currentTile->cell_xform << std::endl;
	//TileBlock newBlock = TileBlock(REGION_BD, currentTile);
	std::cout << "Where is dat 3 in da cell_xform?" << std::endl;
	std::cout << (currentTile->cell_xform.array() == 3) << std::endl;
	std::cout << "MATLAB ismember equivalent: (cell_xform.array()==2).all()" << std::endl;
	std::cout << (currentTile->cell_xform.array()==3).all() << std::endl;
	std::cout << (currentTile->cell_xform.row(2).array()==3).any() << std::endl;
	std::cout << "Generate list of indices from Linspaced" << std::endl;
	std::cout << VectorXi::LinSpaced(10,1,10).transpose() << std::endl;

	std::cout << "Random matrix (vector)" << std::endl;
	MatrixXi Randy = MatrixXi::Random(1,10);
	std::cout << Randy << std::endl;

	std::cout << "Test QR decomp for Least Squaresies" << std::endl;
	MatrixXd A = MatrixXd::Random(3,2);
	std::cout << "Here is a matrix A: \n" << A << std::endl;
	VectorXd b = VectorXd::Random(3);
	std::cout << "Here is the right hand side b:\n" << b << std::endl;
	std::cout << "Solution using QR decomposition: " << std::endl;
	//std::cout << A.colPivHouseholderQr().solve(b) << std::endl;
	Eigen::ColPivHouseholderQR<MatrixXd> dec(A);
	MatrixXd x = dec.solve(b);
	std::cout << x << std::endl;
	int n = 2;
	RowVectorXd x_data(6), y_data(6);
	x_data << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
	y_data << 3.3, 4.5, 6.5, 5.7, 4.3, 2.7;
	VectorXd x_data_trans = x_data.transpose();
	VectorXd y_data_trans = y_data.transpose();
	std::cout << "x_data:\n" << x_data << std::endl;
	MatrixXd V = MatrixXd::Zero(x_data_trans.rows(),n+1);
	V.col(n) = VectorXd::Ones(x_data_trans.rows());
	std::cout << "Made right cols of Ones" << std::endl;
	std::cout << V << std::endl;
	for (int j = n; j > 0; j--){
		V.col(j-1) = x_data_trans.cwiseProduct(V.col(j));
	}
	std::cout << "Vandermonde V:" << std::endl;
	std::cout << V << std::endl;

	Eigen::ColPivHouseholderQR<MatrixXd> myqr(V);
	VectorXd p = myqr.solve(y_data_trans);
	std::cout << "coefficients p: " << std::endl;
	std::cout << p << std::endl;

	return 0;
}
