/*
 * defn_tile.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: bscooper
 *
 *  Functions for Tile class
 */

// Eigen header
//#include <eigen3/Eigen/Core>

#include "trans_sys/hcost/hcost_tile_library.h"

using namespace Eigen;
using namespace srcl;

/* ------- Tile constructors and destructors -------- */
Tile::Tile(int H, Matrix<int,Dynamic,4> tile_vertices){
	FACE_REF <<   1, -1, 1, 0, 0,
					 -2,  2, 1, 2, 0,
					  2, -2, 1, 1, 0,
					 -1,  1, 1, 4, 0,
					  1, -2, 2, 0, 0,
					 -2, -1, 2, 2, 0,
					  2,  1, 2, 1, 0,
					 -1, -2, 2, 4, 0,
					  1,  2, 2, 3, 0,
					  2, -1, 2, 4, 2,
					 -2,  1, 2, 4, 1,
					 -1,  2, 2, 4, 3;
	VERTICES_PERMUTATION << 1, 2, 3, 4,
								4, 1, 2, 3,
								2, 3, 4, 1,
								4, 3, 2, 1,
								2, 1, 4, 3;
	INVERSE_XFORM << 0,2,1,3,4;
	set_tile_data(H,tile_vertices);
	//std::shared_ptr<TileBlock> tile_block;
}

Tile::~Tile(){

}

void Tile::set_tile_data(int H, Matrix<int,Dynamic,4> tile_vertices){
	// ---- Location of cells
	Matrix<double,4,4> I4 = MatrixXd::Identity(4,4);
	traversal_type = MatrixXi::Zero(H,1);  // Opposite (1), adjacent (2)
	cell_xform 	   = MatrixXi::Zero(H,2); // Transformations to bring to standard

	MatrixXi chan_mid(H,4);
	MatrixXi middiag = tile_vertices.block(1,2,H,1).asDiagonal();
	chan_mid.leftCols(2) << middiag*MatrixXi::Ones(H,2); //rect dims (dx,dy)
	chan_mid.rightCols(2) << tile_vertices.block(1,0,H,2);                                //rect center coords
//	std::cout << "chan_mid" << std::endl << chan_mid << std::endl;

	MatrixXd chan_full(H+2,4);
	MatrixXi fulldiagint = tile_vertices.col(2).asDiagonal();
	MatrixXd fulldiag = fulldiagint.cast<double>();
	chan_full.leftCols(2) = fulldiag*MatrixXd::Ones(H+2,2); //Full channel
	chan_full.rightCols(2) = tile_vertices.leftCols(2).cast<double>();						// needed only to define entry/exit segments
//	std::cout << "chan_full" << std::endl << chan_full << std::endl;
	channel_data = chan_mid;
	// Entry segments
	MatrixXi u1(H,1);
	u1 = MatrixXi::Zero(H+1,1);
	MatrixXi  exit_seg(H+1,1);
	exit_seg  = MatrixXi::Zero(H+1,1);
	cell_edge = MatrixXd::Zero(H+1,4);

	for(int n=1; n < H+2; n++){
		double del_X = chan_full(n,2) - chan_full(n-1,2);
		double tol = 1e-6;

		if (abs(abs(del_X) - chan_full.block(n-1,0,2,1).sum()/2.0) < tol){
			//std::cout << "X transition" << std::endl;
			if (chan_full(n,2) > chan_full(n-1,2)){					// X transition
				exit_seg(n-1,0) = -1;								// Right
			}else{
				exit_seg(n-1,0) = 1;								// Left
			}
			u1(n-1,0) = 0;
			double x_lim = chan_full(n-1,2) - exit_seg(n-1,0)*chan_full(n-1,0)/2.0;
			Vector2d y_lim, y_lim_min, y_lim_max;
			y_lim_min << chan_full(n,3) + chan_full(n,1)/2, chan_full(n-1,3) + chan_full(n-1,1)/2;
			y_lim_max << chan_full(n,3) - chan_full(n,1)/2, chan_full(n-1,3) - chan_full(n-1,1)/2;
			y_lim << y_lim_min.minCoeff(), y_lim_max.maxCoeff();
			cell_edge.row(n-1) << x_lim, y_lim(0), x_lim, y_lim(1);
		}else{
			//std::cout << "Y transition" << std::endl;
			if (chan_full(n,3) > chan_full(n-1,3)){					// Y transition
				exit_seg(n-1,0) = 2;								// Up
			}else{
				exit_seg(n-1,0) = -2;								// Down
			}
			u1(n-1,0) = 1;
			double y_lim = chan_full(n-1,3) + exit_seg(n-1,0)/2.0*chan_full(n-1,1)/2.0;
			Vector2d x_lim, x_lim_min, x_lim_max;
			x_lim_min << chan_full(n,2) + chan_full(n,0)/2, chan_full(n-1,2) + chan_full(n-1,0)/2;
			x_lim_max << chan_full(n,2) - chan_full(n,0)/2, chan_full(n-1,2) - chan_full(n-1,0)/2;
			x_lim << x_lim_min.minCoeff(), x_lim_max.maxCoeff();

			cell_edge.row(n-1) << x_lim(0), y_lim, x_lim(1), y_lim;
		}

	}
	VectorXi face_from = -exit_seg.topRows(H);
	VectorXi face_to   = exit_seg.bottomRows(H);\
	traversal_faces.resize(2*H,1);
	traversal_faces << face_from, face_to;
//	std::cout << face_from << std::endl;
//	std::cout << face_to << std::endl;

	// Type of transition
	cell_vertices = MatrixXd::Zero(4,2*H);   // Vertices of each rectangle, side by side
//	std::cout << "cell_vertices" << std::endl;
//	std::cout << cell_vertices << std::endl;
	for (int n=0; n < H; n++){
		Matrix<double,4,2> this_cell_vertices;
		MatrixXd chan_mid_d(H,4);
		chan_mid_d = chan_mid.cast<double>();
		this_cell_vertices.row(0) << chan_mid_d(n,2) - chan_mid_d(n,0)/2.0, chan_mid_d(n,3) + chan_mid_d(n,1)/2.0; // Vertices of the // rectangle in CW
		this_cell_vertices.row(1) << chan_mid_d(n,2) + chan_mid_d(n,0)/2.0, chan_mid_d(n,3) + chan_mid_d(n,1)/2.0; // order starting // with top left
		this_cell_vertices.row(2) << chan_mid_d(n,2) + chan_mid_d(n,0)/2.0, chan_mid_d(n,3) - chan_mid_d(n,1)/2.0;
		this_cell_vertices.row(3) << chan_mid_d(n,2) - chan_mid_d(n,0)/2.0, chan_mid_d(n,3) - chan_mid_d(n,1)/2.0;
//		std::cout << "this_cell_vertices" << std::endl;
//		std::cout << this_cell_vertices << std::endl;
		Matrix<int,1,2> face;
		int idx1;
		for (int mm = 0; mm < 12; mm++){
			face(0,0) = face_from(n);
			face(0,1) = face_to(n);
			if(face.isApprox(FACE_REF.row(mm).leftCols(2))){
				idx1 = mm;
				break;
			}
		}
//		std::cout << "idx1 = " << idx1 << std::endl;
		cell_xform.row(n) = FACE_REF.row(idx1).rightCols(2); // cols 4,5, Existing transformation on current rectangle
//		std::cout << "cell_xform" << std::endl;
//		std::cout << cell_xform << std::endl;
		traversal_type(n,0) = (u1(n,0)!=u1(n+1,0)) + 1;
//		std::cout << "traversal_type" << std::endl;
//		std::cout << traversal_type << std::endl;

		int xform1 = INVERSE_XFORM(FACE_REF(idx1,4));
		int xform2 = INVERSE_XFORM(FACE_REF(idx1,3));
//		std::cout << "xform1 = " << xform1 << std::endl;
//		std::cout << "xform2 = " << xform2 << std::endl;
		RowVectorXi p1 = VERTICES_PERMUTATION.row(xform1);
		RowVectorXi p2 = VERTICES_PERMUTATION.row(xform2);
//		std::cout << "p1 = " << p1 << std::endl;
//		std::cout << "p2 = " << p2 << std::endl;

		Matrix4d I4p1;
		int p10 = p1(0); int p11 = p1(1); int p12 = p1(2); int p13 = p1(3);
		I4p1.row(0) = I4.row(p10-1);
		I4p1.row(1) = I4.row(p11-1);
		I4p1.row(2) = I4.row(p12-1);
		I4p1.row(3) = I4.row(p13-1);
		//std::cout << "I4p1" << std::endl << I4p1 << std::endl;
		Matrix4d I4p2;
		int p20 = p2(0); int p21 = p2(1); int p22 = p2(2); int p23 = p2(3);
		I4p2.row(0) = I4.row(p20-1);
		I4p2.row(1) = I4.row(p21-1);
		I4p2.row(2) = I4.row(p22-1);
		I4p2.row(3) = I4.row(p23-1);
		//std::cout << "I4p2" << std::endl << I4p2 << std::endl;
		MatrixXd this_cell_aug_vertices;
		this_cell_aug_vertices = I4p2*I4p1*this_cell_vertices;
		//std::cout << this_cell_aug_vertices << std::endl;
		cell_vertices.block(0,2*n,4,2) = this_cell_aug_vertices;
	}

}

void Tile::addTileBlock(REGION_BD &REGION_BD, std::shared_ptr<TileBlock> this_tile_block, int Hin){
	//tile_block = std::make_shared<TileBlock>(REGION_BD, this_tile, Hin);
	tile_block = this_tile_block;
}


