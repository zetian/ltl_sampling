/*
 * defn_TileBlock.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: ben
 */

#include "trans_sys/hcost/hcost_tile_library.h"

using namespace Eigen;
using namespace srcl;
TileBlock::TileBlock(){H = 1;}
TileBlock::TileBlock(REGION_BD &REGION_BD, std::shared_ptr<Tile> linked_tile, int Hin) {
	tile = linked_tile; // link this TileBlock with it's Tile
	// access fields tile->cell_vertices,...
	H = Hin;
	// Initialize working variables
	//alfa = MatrixXd::Zero(2*H,N_CBTA_W);

	//std::cout << "N_CBTA_W_SOL = " << N_CBTA_W_SOL << std::endl;
}

TileBlock::~TileBlock() {
	// TODO Auto-generated destructor stub
}

// Main driver of Curvature Bounded Traversability Analysis (CBTA)
//
void TileBlock::cbta(){
	std::cout << "Enter cbta()" << std::endl;
	y_exit = MatrixXd::Zero(H,1);
	z_exit = MatrixXd::Zero(H,1);
	w_upper = MatrixXd::Zero(H,1);
	w_lower = MatrixXd::Zero(H,1);

	w_sol = MatrixXd::Zero(H,N_CBTA_W_SOL);
	w_smp = MatrixXd::Zero(H,N_CBTA_W);
	x_smp = MatrixXd::Zero(H,N_CBTA_W);
	bta_smp = MatrixXd::Zero(2*H,N_CBTA_W);

	bta_smp.row(2*H-2) =  PI/2*MatrixXd::Ones(1,N_CBTA_W);
	bta_smp.row(2*H-1)   = -PI/2*MatrixXd::Ones(1,N_CBTA_W);

	alfa_sol = MatrixXd::Zero(2*H,N_CBTA_W_SOL);
	alfa_smp = MatrixXd::Zero(2*H,N_CBTA_W);
	for (int hlevel = 1; hlevel <= H; hlevel++){ //
		alfa_smp.row(2*hlevel-2) = -T_INFINITY*MatrixXd::Ones(1,N_CBTA_W);
		alfa_smp.row(2*hlevel-1) =  T_INFINITY*MatrixXd::Ones(1,N_CBTA_W);
	}

	for (int nn = 0; nn < H; nn++){
		double y1, y2;
		switch (tile->traversal_type(nn)){
			case 1: {// Measure distance from C for y, z; traversal across face
				Matrix<double,Dynamic,Dynamic> y1temp = tile->cell_vertices.block(2,2*nn,1,2) - tile->cell_edge.block(nn+1,0,1,2);
				Matrix<double,Dynamic,Dynamic> y2temp = tile->cell_vertices.block(2,2*nn,1,2) - tile->cell_edge.block(nn+1,2,1,2);
				y1 = y1temp.norm();
				y2 = y2temp.norm();
				break;
			}
			case 2: {// Measure distance from D for y, z; traversal to adjacent face
				Matrix<double,Dynamic,Dynamic> y1temp = tile->cell_vertices.block(3,2*nn,1,2) - tile->cell_edge.block(nn+1,0,1,2);
				Matrix<double,Dynamic,Dynamic> y2temp = tile->cell_vertices.block(3,2*nn,1,2) - tile->cell_edge.block(nn+1,2,1,2);
				y1 = y1temp.norm();
				y2 = y2temp.norm();
				break;
			}
		}
		y_exit(nn,0) = std::min(y1,y2);
		z_exit(nn,0) = std::max(y1,y2);
	}

	for (int nn = 0; nn < H; nn++){ // Measure distance from D for wL, wU
		Matrix<double,Dynamic,Dynamic> w1temp = tile->cell_vertices.block(3,2*nn,1,2) - tile->cell_edge.block(nn,0,1,2);
		Matrix<double,Dynamic,Dynamic> w2temp = tile->cell_vertices.block(3,2*nn,1,2) - tile->cell_edge.block(nn,2,1,2);

		double w1 = w1temp.norm();
		double w2 = w2temp.norm();
		w_lower(nn,0) = std::min(w1,w2);
		w_upper(nn,0) = std::max(w1,w2);
	}

	for (int nn = 0; nn < H; nn++){
		double w_sol_min;
		if (tile->traversal_type(nn,0)==2){
			double w_sol_temp = w_lower(nn,0);
			w_sol_min = std::max(w_sol_temp,0.001);
		}else{
			w_sol_min = w_lower(nn,0);
		}
		double w_sol_max = w_upper(nn,0); // temp vars because LinSpaced needs
		double y_exit_temp = y_exit(nn,0);// to know size at compile time
		double z_exit_temp = z_exit(nn,0);
		w_sol.row(nn) = VectorXd::LinSpaced(N_CBTA_W_SOL,w_sol_min,w_sol_max).transpose();
		w_smp.row(nn) = VectorXd::LinSpaced(N_CBTA_W,(double)w_lower(nn,0),w_sol_max).transpose();
		x_smp.row(nn) = VectorXd::LinSpaced(N_CBTA_W,y_exit_temp,z_exit_temp).transpose();
	}
//	std::cout << "w_sol" << std::endl << w_sol << std::endl;

	// function pointer 'fcn_cone' is selected and accepts same parameters
	void (srcl::TileBlock::*fcn_cone)(double w, double d,
			//Matrix<double,1,N_CBTA_W>& xSmp,
			RowVectorXd& xSmp,
			Matrix<double,2,N_CBTA_W>& btaSmp,
			Matrix<double,2,1>& returnMatrix);
	if (tile->traversal_type(H-1,0)==1)
		fcn_cone = &srcl::TileBlock::cbta_s1;
	else
		fcn_cone = &srcl::TileBlock::cbta_s2;

	for (int qq = 0; qq < N_CBTA_W_SOL; qq++){
//		std::cout << "qq = " << qq << std::endl;
		double w = w_sol(H-1,qq);
		double d = tile->channel_data(H-1,0);
		//Matrix<double,1,N_CBTA_W> xSmp = x_smp.row(H-1);
		RowVectorXd xSmp = x_smp.row(H-1);
		Matrix<double,2,N_CBTA_W> btaSmp = bta_smp.bottomRows(2);
		Matrix<double,2,1> alfa;
		alfa << -T_INFINITY, T_INFINITY;
		(this->*fcn_cone)(w,d,xSmp,btaSmp,alfa);
		alfa_sol.block(2*H-2,qq,2,1) = alfa; // bottom 2 rows, qqth column in 2*H by N_CBTA_W_SOL matrix
	}

	// ------ Interpolate solved alfa over the whole grid
	RowVectorXd w_sol_row = w_sol.row(H-1);
	Matrix<double,2,Dynamic> alfa_sol_rows = alfa_sol.bottomRows(2);
	RowVectorXd w_smp_row = w_smp.row(H-1);
	Matrix<double,2,Dynamic> y_interp;
	interp_broken_seg(w_sol_row,alfa_sol_rows,w_smp_row,y_interp);
	alfa_smp.bottomRows(2) = y_interp;

	for (int pp = H-1; pp > 0; pp--){
		// -------- Transform alfa of previous to get bta for current
		// Many ismember operations here. Look into Eigen Matrix vs. Array comparison operators, possibly 'select' function
		if (((tile->cell_xform.row(pp).array()==3).any() && !(tile->cell_xform.row(pp).array()==4).any())
		 || (!(tile->cell_xform.row(pp).array()==3).any() && (tile->cell_xform.row(pp).array()==4).any())){
			Matrix<double,2,N_CBTA_W> alfa_smp_rows = alfa_smp.middleRows(2*pp,2); // If flipped once
			bta_smp.middleRows(2*pp-2,2) = (-alfa_smp_rows.colwise().reverse()).rowwise().reverse(); //fliplr(-flipud(A))
		}else
			bta_smp.middleRows(2*pp-2,2) = alfa_smp.middleRows(2*pp,2);
		if (((tile->cell_xform.row(pp-1).array()==3).any() && !(tile->cell_xform.row(pp-1).array()==4).any())
		 || (!(tile->cell_xform.row(pp-1).array()==3).any() && (tile->cell_xform.row(pp-1).array()==4).any())){
			Matrix<double,2,N_CBTA_W> bta_smp_rows = bta_smp.middleRows(2*pp-2,2); // If flipped once
			bta_smp.middleRows(2*pp-2,2) = (-bta_smp_rows.colwise().reverse()).rowwise().reverse(); //fliplr(-flipud(B))
		}
		// ---------- Solve for target sets
		if (tile->traversal_type(pp-1,0)==1)
			fcn_cone = &srcl::TileBlock::cbta_s1;
		else
			fcn_cone = &srcl::TileBlock::cbta_s2;

		for (int qq = 0; qq < N_CBTA_W_SOL; qq++){
				double w = w_sol(pp-1,qq);
				double d = tile->channel_data(pp-1,0);
				RowVectorXd xSmp = x_smp.row(pp-1);
				Matrix<double,2,N_CBTA_W> btaSmp = bta_smp.middleRows(2*pp-2,2);
				Matrix<double,2,1> alfa;
				alfa << -T_INFINITY, T_INFINITY;
				(this->*fcn_cone)(w,d,xSmp,btaSmp,alfa);
//				std::cout << "Exited cbta_s1 or cbta_s2" << std::endl;
//				std::cout << "returned alfa value" << std::endl << alfa << std::endl;
				alfa_sol.block(2*pp-2,qq,2,1) = alfa; // bottom 2 rows, qqth column in 2*H by N_CBTA_W_SOL matrix
			}

		// ------ Interpolate over all x
		RowVectorXd w_sol_row = w_sol.row(pp-1);
		Matrix<double,2,Dynamic> alfa_sol_rows = alfa_sol.middleRows(2*pp-2,2);
		RowVectorXd w_smp_row = w_smp.row(pp-1);
		Matrix<double,2,Dynamic> y_interp;
		interp_broken_seg(w_sol_row,alfa_sol_rows,w_smp_row,y_interp);
		alfa_smp.middleRows(2*pp-2,2) = y_interp;
	}
//	std::cout << "cbta: Made it past Interpolate over all x" << std::endl;
	alfa = alfa_smp;
	bta = bta_smp;
	x = x_smp;
	w = w_smp;

}

// Implmented from libigl - sortrows
// see https://github.com/libigl/libigl/blob/master/include/igl/sortrows.cpp
// X is original matrix, Y is sorted matrix, IX is column to sort by
// ascending = true for top row lowest IX col value
// No support for MatrixBase<Ducks>, unable to place ducks in a row
template <typename DerivedX, typename DerivedIX>
void sortrows(const PlainObjectBase<DerivedX>& X, const bool ascending,
		      PlainObjectBase<DerivedX>& Y, PlainObjectBase<DerivedIX>& IX){
	const size_t num_rows = X.rows();
	const size_t num_cols = X.cols();
	Y.resize(num_rows,num_cols);
	IX.resize(num_rows,1);
	for (int i = 0; i < num_rows; i++){
		IX(i) = i;
	}
	if (ascending){
		auto index_less_than = [&X, num_cols](size_t i, size_t j){
			for (size_t c = 0; c < num_cols; c++){
				if (X.coeff(i,c) < X.coeff(j,c)) return true;
				else if (X.coeff(j,c) < X.coeff(i,c)) return false;
			}
			return false;
		};
		std::sort(IX.data(), IX.data() + IX.size(), index_less_than);
	}else {
		auto index_greater_than = [&X, num_cols](size_t i, size_t j){
			for (size_t c = 0; c < num_cols; c++){
				if (X.coeff(i,c) > X.coeff(j,c)) return true;
				else if (X.coeff(j,c) > X.coeff(i,c)) return false;
			}
			return false;
		};
		std::sort(IX.data(), IX.data() + IX.size(), index_greater_than);
	}
	for (size_t j = 0; j < num_cols; j++){
		for (int i = 0; i < num_rows; i++){
			Y(i,j) = X(IX(i),j);
		}
	}
}

void TileBlock::cbta_s1(double w, double d,
		                //Matrix<double,1,N_CBTA_W>& xSmp,
		                RowVectorXd& xSmp,
		                Matrix<double,2,N_CBTA_W>& btaSmp,
		                Matrix<double,2,1>& return_alfa){
//	std::cout << "---- Enter cbta_s1 ----" << std::endl;
	// ------ Max and min initial angle possible
//	std::cout << "w = " << w << std::endl;
//	std::cout << "d = " << d << std::endl;
	double alfaStL = -std::acos(1-w/r_min);
	double alfaStU =  std::acos(1-(d-w)/r_min);
//	std::cout << "alfaStL = " << alfaStL << std::endl;
//	std::cout << "alfaStU = " << alfaStU << std::endl;

	// ------ Intersect with segment with non-Inf bta
	//Matrix<int,1,Dynamic> btaNotInfIndx;
	RowVectorXi btaNotInfIndx;
	//Matrix<double,1,N_CBTA_W> btaSmp_row = btaSmp.row(0);
	RowVectorXd btaSmp_row = btaSmp.row(0);
	remove_inf_values(btaSmp_row,btaNotInfIndx);
	if ((int)btaNotInfIndx(0) == -1){ //check for -1: if index empty, all vals +/- Inf
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	// -------- Portion of exit segment reachable by C arcs alone
	// Uppermost point directly reachable by C- assumed to be B because r >= d,
	// and n2L = w + sqrt(2*r*d - d^2)
	// Lowermost point directly reachable by C+ assumed to be C because r >= d,
	// and n1U = w - sqrt(2*r*d - d^2)
	double smpN1L, smpN2U;
//	std::cout << std::pow((d - r_min*std::sin(-alfaStL)),2) << std::endl;
	if (r_min < (std::pow(d,2) + std::pow(w,2))/2.0/w){ // Lowermost point directly reachable by C-
		double n1L = r_min - std::sqrt(std::pow(r_min,2) - std::pow((d - r_min*std::sin(-alfaStL)),2));
		smpN1L = std::min(std::max(find_sample(xSmp,n1L),(int)btaNotInfIndx(0)),btaNotInfIndx.tail(1).value());
	}else{
		smpN1L = (int)btaNotInfIndx(0);
	}
//	std::cout << "n1L calculated" << std::endl;
	if (r_min < (std::pow(d,2) + std::pow(d-w,2))/2.0/(d-w)){ // Lowermost point directly reachable by C-
		double n2U = r_min - d + std::sqrt(std::pow(r_min,2) - std::pow((d - r_min*std::sin(alfaStU)),2));
		smpN2U = std::max(std::min(find_sample(xSmp,n2U),btaNotInfIndx.tail(1).value()),(int)btaNotInfIndx(0));
	}else{
		smpN2U = btaNotInfIndx.tail(1).value();
	}
//	std::cout << "n2U calculated" << std::endl;
	if (smpN2U <= smpN1L){
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	// ---------- Calculates angles g+ and g-
	Matrix<double,1,Dynamic> gamPSmp = MatrixXd::Zero(1,(int)btaNotInfIndx.cols()); // Lowest angle possible at X, by C+
	Matrix<double,1,Dynamic> gamMSmp = MatrixXd::Zero(1,(int)btaNotInfIndx.cols()); // Highest angle possible at X, by C-

	for (int m = (int)btaNotInfIndx(0); m < smpN1L; m++){
		double x = xSmp(0,m);
		double C0 = std::pow(d,2) + std::pow(x-w,2);
		double C1 = std::sqrt(4*std::pow(r_min,2)*C0 - std::pow(C0,2));
		double C2 = -2*r_min*(x-w) - C0;
		double gamP = 2*std::atan((2*r_min*d - C1)/C2); // Angle at X of C+ arc from W to X

		double gamM = std::acos(1 - x/r_min);       // Angle at X of C- arc tangent to DC passing through X

		gamPSmp(0,m - (int)btaNotInfIndx(0)) = gamP;
		gamMSmp(0,m - (int)btaNotInfIndx(0)) = gamM;
	}

	for (int m = smpN1L; m <= smpN2U; m++){
		double x = xSmp(0,m);
		double C0 = std::pow(d,2) + std::pow(x-w,2);
		double C1 = std::sqrt(4*std::pow(r_min,2)*C0 - std::pow(C0,2));
		double C2 = -2*r_min*(x-w) - C0;
		double gamP = 2*std::atan((2*r_min*d - C1)/C2); // Angle at X of C+ arc from W to X

		double gamM = PI + 2*std::atan((2*r_min*d + C1)/C2); // Angle at X of C- arc from W to X
		if (gamM > PI){
			gamM = gamM - 2*PI;
		}
		gamPSmp(0,m - (int)btaNotInfIndx(0)) = gamP;
		gamMSmp(0,m - (int)btaNotInfIndx(0)) = gamM;
	}

	for (int m = smpN2U+1; m <= btaNotInfIndx.tail(1).value(); m++){
		double x = xSmp(0,m);
		double gamP = -std::acos(1 - (d-x)/r_min); // Angle at X of C+ arc tangent to AB passing through X\

		double C0 = std::pow(d,2) + std::pow(x-w,2);
		double C1 = std::sqrt(4*std::pow(r_min,2)*C0 - std::pow(C0,2));
		double C2 = -2*r_min*(x-w) - C0;
		double gamM = pi2pi(PI + 2*std::atan(2*r_min*d + C1)/C2);

		gamPSmp(0,m - (int)btaNotInfIndx(0)) = gamP;
		gamMSmp(0,m - (int)btaNotInfIndx(0)) = gamM;
	}

	// -------- Find critical points (works because of continuity)
	// Need to index into btaSmp using btaNotInfIndx's, Indexing
//	Matrix<double,1,Dynamic> diffPU;
//	Matrix<double,1,Dynamic> diffPL;
//	Matrix<double,1,Dynamic> diffML;
//	Matrix<double,1,Dynamic> diffMU;
	RowVectorXd diffPU; diffPU.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffPL; diffPL.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffMU; diffMU.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffML; diffML.resize(1,btaNotInfIndx.cols());
	for (int ind = 0; ind < (int)btaNotInfIndx.cols(); ind++){
		diffPU(ind) = gamPSmp(0,ind) - btaSmp(0,(int)btaNotInfIndx(ind));
		diffPL(ind) = gamPSmp(0,ind) - btaSmp(1,(int)btaNotInfIndx(ind));
		diffMU(ind) = gamMSmp(0,ind) - btaSmp(0,(int)btaNotInfIndx(ind));
		diffML(ind) = gamMSmp(0,ind) - btaSmp(1,(int)btaNotInfIndx(ind));
	}

	RowVectorXi xPU;
	RowVectorXi xML;
	find_zeros(diffPU,xPU);
	find_zeros(diffML,xML);

	RowVectorXi xC;
	if ((int)xPU(0) != -1 && (int)xML(0) != -1){ // if both xPU and xML non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xPU.cols() + xML.cols());
		x_joined << xPU, xML;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size()-1;
	}else if((int)xPU(0) == -1 && (int)xML(0) != -1){ // only xML non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xML.cols());
		x_joined << xML;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size()-1;
	}else if((int)xPU(0) != -1 && (int)xML(0) == -1){      // only xPU non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xPU.cols());
		x_joined << xPU;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size()-1;
	}else{                                         // xPU and xML empty
		xC.resize(1,2);
		xC << -1, btaNotInfIndx.size()-1;
	}
	// ------ Find type of interval
	// Note: because of continuity, one sample of each interval suffices
	// to figure out which "category" the whole interval belongs to
	RowVectorXi intMark;
	std::vector<int> marks;
	for (int m = 0; m < xC.size()-1; m++){
		if ((diffPU(xC(m) + 1) > 0) || (diffML(xC(m) + 1) < 0)){ // g+ > btu_U or g- < bta_L; no solution
			marks.push_back(0);
		}else{
			marks.push_back(1);
		}
	}
	intMark = RowVectorXi::Map(marks.data(),marks.size());

	// ------- Find largest interval in N1-N2
	Matrix<int,Dynamic,Dynamic> intsGood;
	if (intMark(0)){
		intsGood.resize(1,2);
		intsGood.row(0) << xC(0)+1, xC(1);
	}//else intsGood starts empty
	for (int m = 1; m < xC.size()-1; m++){
		if (intMark(m) && intMark(m-1))
			intsGood(intsGood.rows()-1,1) = xC(m+1);
		else if (intMark(m) && !intMark(m-1)){
			intsGood.conservativeResize(intsGood.rows()+1,2);
			intsGood.row(intsGood.rows()-1) << xC(m)+1, xC(m+1);
		}
	}
	if (intsGood.rows()==0){ //intsGood remained empty
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	intsGood.conservativeResize(NoChange,intsGood.cols()+1);
	intsGood.col(intsGood.cols()-1) = intsGood.col(1) - intsGood.col(0);

	//Matrix<int,Dynamic,3> intsGoodHack = intsGood;
	VectorXi intsGoodDiff = intsGood.col(2);
	Matrix<int,Dynamic,Dynamic> intsGoodSorted;
	sortrows(intsGood,true,intsGoodSorted,intsGoodDiff); //sort by 3rd col in ascending order
	int xIntMaxSmp1 = intsGoodSorted(0,0);
	int xIntMaxSmp2 = intsGoodSorted(0,1);

	// ------- Man and min of Ux' and Lx' on interval found above
	RowVectorXi intSmp = RowVectorXi::LinSpaced(xIntMaxSmp2-xIntMaxSmp1+1,xIntMaxSmp1,xIntMaxSmp2);

	if (intSmp.size()==0){
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}
	VectorXd thtaPSmp = VectorXd::Zero(intSmp.size());
	VectorXd thtaMSmp = VectorXd::Zero(intSmp.size());

	int n = -1;
	std::vector<int> intSmp_vec(intSmp.data(),intSmp.data() + intSmp.size());
		for (auto m : intSmp_vec){
			n++;
			double thtaP, thtaM;
			double x = xSmp(m + btaNotInfIndx(0)); // -2 for 0-indexing? no (FFW to first notInf sample)
			if (diffPL(m) < 0){
				double bta = btaSmp(1,m + btaNotInfIndx(0)); // same -2? no
				double C0 = pow(d,2) + pow(x-w,2);
				double A = sin(bta)	- d/r_min;
				double B = cos(bta) + (x-w)/r_min;
				double C = -((x-w)*cos(bta)/r_min - d*sin(bta)/r_min + C0/(2*pow(r_min,2)) - 1);
				thtaP = 2*atan((A + sqrt(pow(A,2) + pow(B,2) - pow(C,2)))/(B+C));
			}else{
				if (m + btaNotInfIndx(0) <= smpN2U)
					thtaP = gamMSmp(m); // correct for 0-indexing?, was not, now corrected, corrected back
				else
					thtaP = alfaStU;
			}
			thtaPSmp(n) = thtaP;

			if (diffMU(m) > 0){
				double bta = btaSmp(0,m + btaNotInfIndx(0)); // -2 for 0-indexing? yes
				double C0 = pow(d,2) + pow(x-w,2);
				double A = sin(bta) + d/r_min;
				double B = cos(bta) - (x-w)/r_min;
				double C = -(-(x-w)*cos(bta)/r_min + d*sin(bta)/r_min + C0/(2*pow(r_min,2)) - 1);
				thtaM = 2*atan((A - sqrt(pow(A,2) + pow(B,2) - pow(C,2)))/(B+C));
			}else{
				if (m + btaNotInfIndx(0) >= smpN1L)
					thtaM = gamPSmp(m); //correct for 0-indexing?
				else
					thtaM = alfaStL;
			}
			thtaMSmp(n) = thtaM;
		}
		return_alfa << thtaPSmp.maxCoeff(), thtaMSmp.minCoeff();
}

void TileBlock::cbta_s2(double w, double d,
		                //Matrix<double,1,N_CBTA_W>& xSmp,
		                RowVectorXd& xSmp,
		                Matrix<double,2,N_CBTA_W>& btaSmp,
		                Matrix<double,2,1>& return_alfa){
//	std::cout << "**** Enter cbta_s2 ****" << std::endl;
	// ------ Intersect with segment with non-Inf bta
	RowVectorXi btaNotInfIndx;
	RowVectorXd btaSmp_row = btaSmp.row(0);
	remove_inf_values(btaSmp_row,btaNotInfIndx);

	if ((int)btaNotInfIndx(0) == -1){ //check for -1: if index empty, all vals +/- Inf
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	// ------- Portion of exit segment reachable by C arcs alone
	// Rightmost point directly reachable by C- assumed to be C
	// Leftmost point directly reacable by C+ assumed to be D
	// Rightmost point directly reachable by C+ assumed to be C
	double n1L = r_min - sqrt(pow(r_min,2) - pow(w,2)); // Leftmost point directly reachable by C-
	double smpN1L = std::min(std::max(find_sample(xSmp,n1L),btaNotInfIndx(0)),btaNotInfIndx.tail(1).value());
	double smpN2U = btaNotInfIndx.tail(1).value();

	if (smpN2U <= smpN1L){
			return_alfa << -T_INFINITY, T_INFINITY;
			return;
	}

	RowVectorXd gamPSmp = RowVectorXd::Zero(btaNotInfIndx.size()); // lowest angle possible at X, by C+
	RowVectorXd gamMSmp = RowVectorXd::Zero(btaNotInfIndx.size()); // highest angle possible at X, by C-

	for (int m = btaNotInfIndx(0); m < smpN1L; m++){
		double x = xSmp(m);
		double C0 = std::pow(x,2) + std::pow(w,2);
		double C1 = std::sqrt(4*std::pow(r_min,2)*C0 - std::pow(C0,2));
		double C2 = 2*r_min*x + C0;
		double gamP = 2*std::atan((-2*r_min*w + C1)/C2); // Angle at X of C+ arc from W to X
		double gamM = std::acos(1 - x/r_min);

		gamPSmp(m - btaNotInfIndx(0)) = gamP;
		gamMSmp(m - btaNotInfIndx(0)) = gamM;
	}

	for (int m = smpN1L; m <= smpN2U; m++){
		double x = xSmp(m);
		double C0 = std::pow(x,2) + std::pow(w,2);
		double C1 = std::sqrt(4*std::pow(r_min,2)*C0 - std::pow(C0,2));
		double C2 = 2*r_min*x + C0;
		double gamP = 2*std::atan((-2*r_min*w + C1)/C2); // Angle at X of C+ arc from W to X
		double gamM = PI + 2*std::atan((-2*r_min*w - C1)/C2); // Angle at X of C- arc from W to X
		if (gamM > PI){gamM = gamM - 2*PI;}

		gamPSmp(m - btaNotInfIndx(0)) = gamP;
		gamMSmp(m - btaNotInfIndx(0)) = gamM;
	}

	// -------- Find critical points (works because of continuity)
	RowVectorXd diffPU; diffPU.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffPL; diffPL.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffML; diffML.resize(1,btaNotInfIndx.cols());
	RowVectorXd diffMU; diffMU.resize(1,btaNotInfIndx.cols());
	for (int ind = 0; ind < btaNotInfIndx.size(); ind++){
		diffPU(ind) = gamPSmp(ind) - btaSmp(0,btaNotInfIndx(ind));
		diffPL(ind) = gamPSmp(ind) - btaSmp(1,btaNotInfIndx(ind));
		diffMU(ind) = gamMSmp(ind) - btaSmp(0,btaNotInfIndx(ind));
		diffML(ind) = gamMSmp(ind) - btaSmp(1,btaNotInfIndx(ind));
	}
	RowVectorXi xPU;
	RowVectorXi xML;
	find_zeros(diffPU,xPU);
	find_zeros(diffML,xML);

	RowVectorXi xC;
	if ((int)xPU(0) != -1 && (int)xML(0) != -1){ // if both xPU and xML non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xPU.cols() + xML.cols());
		x_joined << xPU, xML;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size() - 1;
	}else if((int)xPU(0) == -1 && (int)xML(0) != -1){ // only xML non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xML.cols());
		x_joined << xML;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size() - 1;
	}else if((int)xPU(0) != -1 && (int)xML(0) == -1){      // only xPU non-empty
		RowVectorXi x_joined;
		x_joined.resize(1,xPU.cols());
		x_joined << xPU;
		std::sort(x_joined.data(),x_joined.data() + x_joined.size());
		xC.resize(1,x_joined.size() + 2);
		xC << -1, x_joined, btaNotInfIndx.size() - 1;
	}else{                                         // xPU and xML empty
		xC.resize(1,2);
		xC << -1, btaNotInfIndx.size() - 1;
	}


	// ------- Find type of interval
	// Note: because of continuity, one sample of each interval suffices to
	// figure out which "category" the whole interval belongs to
	RowVectorXi intMark;
	std::vector<int> marks;
	for (int m = 0; m < xC.size()-1; m++){
		if ((diffPU(xC(m) + 1) > 0) || (diffML(xC(m) + 1) < 0)){ // g+ > btu_U or g- < bta_L; no solution
			marks.push_back(0);
		}else{
			marks.push_back(1);
		}
	}
	intMark = RowVectorXi::Map(marks.data(),marks.size());

	// ------- Find largest intervalin N1-N2
	Matrix<int,Dynamic,Dynamic> intsGood;
	if (intMark(0)){
		intsGood.resize(1,2);
		intsGood.row(0) << xC(0)+1, xC(1);
	}//else intsGood starts empty
	for (int m = 1; m < xC.size()-1; m++){
		if (intMark(m) && intMark(m-1))
			intsGood(intsGood.rows()-1,1) = xC(m+1);
		else if (intMark(m) && !intMark(m-1)){
			intsGood.conservativeResize(intsGood.rows()+1,2);
			intsGood.row(intsGood.rows()-1) << xC(m)+1, xC(m+1);
		}
	}
	if (intsGood.rows()==0){ //intsGood remained empty
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	intsGood.conservativeResize(NoChange,intsGood.cols()+1);
	intsGood.col(intsGood.cols()-1) = intsGood.col(1) - intsGood.col(0);
	//Matrix<int,Dynamic,3> intsGoodHack = intsGood;
	VectorXi intsGoodDiff = intsGood.col(2);
	Matrix<int,Dynamic,Dynamic> intsGoodSorted;
	sortrows(intsGood,true,intsGoodSorted,intsGoodDiff); //sort by 3rd col in ascending order
	int xIntMaxSmp1 = intsGoodSorted(0,0);
	int xIntMaxSmp2 = intsGoodSorted(0,1);

	// --------- Max and min of Ux' and Lx' on interval found above
	RowVectorXi intSmp = RowVectorXi::LinSpaced(xIntMaxSmp2-xIntMaxSmp1+1,xIntMaxSmp1,xIntMaxSmp2);

	if (intSmp.size()==0){
		return_alfa << -T_INFINITY, T_INFINITY;
		return;
	}

	VectorXd thtaPSmp = VectorXd::Zero(intSmp.size());
	VectorXd thtaMSmp = VectorXd::Zero(intSmp.size());

	int n = -1;
	std::vector<int> intSmp_vec(intSmp.data(),intSmp.data() + intSmp.size());
	for (auto m : intSmp_vec){
		n++;
		double thtaP, thtaM;
		double x = xSmp(m + btaNotInfIndx(0)); // same -2 for 0-indexing as cbta_s1 questions.
		if (diffPL(m) < 0){
			double bta = btaSmp(1,m + btaNotInfIndx(0));
			double C0 = pow(x,2) + pow(w,2);
			double A = -cos(bta) - x/r_min;
			double B = sin(bta) - w/r_min;
			double C = 1 - (x*cos(bta) - w*sin(bta))/r_min - C0/(2*pow(r_min,2));
			thtaP = 2*atan((A + sqrt(pow(A,2) + pow(B,2) - pow(C,2)))/(B+C));
		}else{
			thtaP = gamMSmp(m) - PI/2.0;
		}
		thtaPSmp(n) = thtaP;

		if (diffMU(m) > 0){
			double bta = btaSmp(0,m + btaNotInfIndx(0));
			double C0 = pow(x,2) + pow(w,2);
			double A = -cos(bta) + x/r_min;
			double B = sin(bta) + w/r_min;
			double C = 1 + (x*cos(bta) - w*sin(bta))/r_min - C0/(2*pow(r_min,2));
			double atanResult = atan((A - sqrt(pow(A,2) + pow(B,2) - pow(C,2)))/(B+C));
			thtaM = std::max(-PI/2.0, 2.0*atanResult);
		}else{
			if (m + btaNotInfIndx(0) >= smpN1L)
				thtaM = gamPSmp(m) - PI/2.0;
			else
				thtaM = -PI/2.0;
		}
		thtaMSmp(n) = thtaM;
	}
	return_alfa << thtaPSmp.maxCoeff(), thtaMSmp.minCoeff();
}

template <typename Derived1, typename Derived2>
void TileBlock::remove_inf_values(MatrixBase<Derived1>& v,
								  MatrixBase<Derived2>& notInfIndex){

//	Matrix<bool,1,Dynamic> infty = (v.template cast<double>()).array()==T_INFINITY +
//					  (v.template cast<double>()).array()==-T_INFINITY;
	ArrayXXd infty_array = (v.template cast<double>()).array();
	//std::cout << (infty_array == T_INFINITY).cast<int>() << std::endl;

	Matrix<double,1,Dynamic> infty1 = (infty_array == T_INFINITY).cast<double>();
	Matrix<double,1,Dynamic> infty2 = (infty_array ==-T_INFINITY).cast<double>();
//	Matrix<double,1,Dynamic> infty = ((v.template cast<double>()).array()==T_INFINITY).cast<int>() +
//			  ((v.template cast<double>()).array()==-T_INFINITY).cast<int>();
	Matrix<int,1,Dynamic> infty = (infty1 + infty2).cast<int>();
	if ((infty.array()==0).all()){ // if all elements are zero (no Inf values)
		notInfIndex = VectorXi::LinSpaced(v.template size(),0,v.template size());
	}

	if ((infty.array()==1).all()){ // if all elements indexed as +/- Infinity
		Matrix<int,1,1> emptyMatrix;
		emptyMatrix << -1;
		notInfIndex = emptyMatrix;// return empty
	}

	// MATLAB version of this uses a recursive function that seems to stitch
	// together the nonInf segments into a contigous array. Not sure if that
	// is worth maintaining for just recording indices of nonInfs.
	std::vector<int> indices;
	for (int inf_it = 0; inf_it < infty.size(); inf_it++){
		if (!infty(0,inf_it)){
			indices.push_back(inf_it);
		}
	}
	notInfIndex = RowVectorXi::Map(indices.data(),indices.size()); // something fixed sized method on dynamic sized vector
//	std::cout << "Exit remove_inf_values" << std::endl;
}

double sign_func(double x){
	if (x>0.0)
		return 1.0;
	else if (x==0.0)
		return 0.0;
	else
		return -1.0;
}
template <typename Derived_a, typename Derived_b>
void TileBlock::find_zeros(MatrixBase<Derived_a>& fSmp, MatrixBase<Derived_b>& idx_zeros){

	Matrix<double,1,Dynamic> f1 = fSmp.template cast<double>();
	MatrixXd f1un = f1.unaryExpr(std::ptr_fun(sign_func));
	f1un.cast<int>();

	std::vector<int> indices;
	for (int diff_it = 0; diff_it < f1un.cols()-1; diff_it++){
		if ((f1un(diff_it+1)-f1un(diff_it))!=0){
			indices.push_back(diff_it);
		}
	}
	if (indices.empty()){
		Matrix<int,1,1> emptyMatrix;
		emptyMatrix << -1;
		idx_zeros = emptyMatrix; // use -1 as an indicator of empty matrix
	}else
		idx_zeros = RowVectorXi::Map(indices.data(),indices.size());

}

std::vector<double> polyfit(RowVectorXd& x_data, RowVectorXd& y_data, const int n){
	// Construct Vandermonde matrix
	VectorXd x_data_trans = x_data.transpose();
	MatrixXd V = MatrixXd::Zero(x_data_trans.rows(),n+1);
	VectorXd p;

	V.col(n) = VectorXd::Ones(x_data_trans.rows());
	for (int j = n; j > 0; j--){
		V.col(j-1) = x_data_trans.cwiseProduct(V.col(j));
	}
	// Use Eigen's QR decomposition
	VectorXd y_data_transpose = y_data.transpose();
	Eigen::ColPivHouseholderQR<MatrixXd> dec(V);
	p = dec.solve(y_data_transpose);
	std::vector<double> p_vec(p.data(), p.data() + p.rows());
	return p_vec;

}

void polyval(std::vector<double> coeffs, RowVectorXd& x_data, RowVectorXd& y_out){
	y_out = RowVectorXd::Zero(y_out.cols());
	int n_poly_order = coeffs.size();
	int p = 0;
	for (int exponent = n_poly_order - 1; exponent >= 0; exponent--){
		y_out.array() += coeffs[p]*x_data.array().pow(exponent);
		p++;
	}
}

//void kron()

void TileBlock::interp_broken_seg(RowVectorXd& x_data,
	                   Matrix<double,2,Dynamic>& y_data,
	                   RowVectorXd& x_interp,
	                   Matrix<double,2,Dynamic>& y_interp){
//	std::cout << "Enter interp_broken_seg" << std::endl;
	int n_poly_order = 5;
	RowVectorXi y_noInf_inds;
	RowVectorXd ydata_row = y_data.row(0);
	remove_inf_values(ydata_row,y_noInf_inds);
	n_poly_order = std::min(n_poly_order,(int)y_noInf_inds.size()-1);

	if ((int)y_noInf_inds.size() > 0){ // index of y_data with no Infs is not empty
		int idx1 = 0;
		int idx2 = 0;
		for (int ifirst = 0; ifirst < x_interp.cols(); ifirst++) // search forward for first match
			if (x_interp(ifirst) >= x_data(y_noInf_inds(0))){
				idx1 = ifirst;
				break;
			}

		for (int isecond = x_interp.cols()-1; isecond >= 0; isecond--) //search backword for last match
			if (x_interp(isecond) <= x_data(y_noInf_inds.tail(1).value())){
				idx2 = isecond;
				break;
			}
		y_interp.resize(2,x_interp.cols());
		y_interp.row(0) = -T_INFINITY*MatrixXd::Ones(1,x_interp.cols());
		y_interp.row(1) =  T_INFINITY*MatrixXd::Ones(1,x_interp.cols());

		RowVectorXd x_data_noInf;
		RowVectorXd y_data_row0; // = y_data.row(0);
		RowVectorXd y_data_row1; // = y_data.row(1);
		std::vector<double> x_data_vec;
		std::vector<double> y_data_vec1;
		std::vector<double> y_data_vec2;
		for (int ind = 0; ind < y_noInf_inds.size(); ind++){
			int noInfInd = y_noInf_inds(ind);
			x_data_vec.push_back(x_data(noInfInd));
			y_data_vec1.push_back(y_data(0,noInfInd));
			y_data_vec2.push_back(y_data(1,noInfInd));
		}
		x_data_noInf = RowVectorXd::Map(x_data_vec.data(),x_data_vec.size());
		y_data_row0  = RowVectorXd::Map(y_data_vec1.data(),y_data_vec1.size());
		y_data_row1  = RowVectorXd::Map(y_data_vec2.data(),y_data_vec2.size());

		std::vector<double> y_poly1 = polyfit(x_data_noInf, y_data_row0, n_poly_order);
		std::vector<double> y_poly2 = polyfit(x_data_noInf, y_data_row1, n_poly_order);

		RowVectorXd x_segment = x_interp.segment(idx1,idx2-idx1+1);
		RowVectorXd y_val1(x_segment.size());
		polyval(y_poly1,x_segment,y_val1);
		y_interp.block(0,idx1,1,idx2-idx1+1) = y_val1;
		RowVectorXd y_val2(x_segment.size());
		polyval(y_poly2,x_segment,y_val2);
		y_interp.block(1,idx1,1,idx2-idx1+1) = y_val2;
	}else{
		// y_interp = kron([-Inf,Inf],ones(1,numel(x_interp)));
		// which for these matrix dimensions is just the same as
		y_interp.resize(2,x_interp.cols());
		y_interp.row(0) = -T_INFINITY*MatrixXd::Ones(1,x_interp.cols());
		y_interp.row(1) =  T_INFINITY*MatrixXd::Ones(1,x_interp.cols());
	}
//	std::cout << "Exit interp_broken_seg" << std::endl;
}

// return index of y if found in ySmp (closest value of ySmp above y)
// or return 1st index (0) if less than ySmp, and max index (size of ySmp)
// if y greater than ySmp
//int TileBlock::find_sample(Matrix<double,1,N_CBTA_W>& ySmp, double y){
int TileBlock::find_sample(RowVectorXd& ySmp, double y){
//	std::cout << "Enter find_sample" << std::endl;
	int sample;
	if (y < ySmp(0)){
		sample = 0;
	}

	int n_cols = ySmp.cols();
//	std::cout << "n_cols = " << n_cols << std::endl;
//	std::cout << "y = " << y << std::endl;
	if (y > ySmp(n_cols-1)){
		sample = ySmp.cols(); //size of vector/matrix
	}
	//std::cout << "Made it here" << std::endl;
	for (int y_iter = 0; y_iter < n_cols; y_iter++){
		if (ySmp(y_iter) >= y){
			sample = y_iter;
			break;
		}
	}
//	std::cout << "Exit find_sample" << std::endl;
	return sample;
}

double TileBlock::pi2pi(double x){ // Return angles between -pi and pi
	double slice;
	while ((x > PI) || (x < -PI)){
		if (x > PI)
			x = x - 2*PI;
		else
			x = x + 2*PI;
	}
	return slice; //of pi
}


// Curvature Bounded Reachability Analysis (cbra)
// Given an index in the region, return all reachable indexes
void TileBlock::cbra(int idx_r_from,REGION_BD &REGION_BD,double r_min,
		             long int N_REGION_TOTAL,
		             RowVectorXi& region_target_2,
		             RowVectorXi& region_neighbors){
//	std::cout << "Enter CBRA" << std::endl;
//	Matrix<int,1,Dynamic> region_neighbors;
	double rgn_from_w_lower = REGION_BD.region_w_lower[idx_r_from];
	double rgn_from_w_upper = REGION_BD.region_w_upper[idx_r_from];
	double rgn_from_psi_lower = REGION_BD.region_psi_lower[idx_r_from];
	double rgn_from_psi_upper = REGION_BD.region_psi_upper[idx_r_from];

	double xL_from_wL_psiL;
	double xU_from_wL_psiL;
	double btaU_from_wL_psiL;
	double xL_from_wU_psiU;
	double btaL_from_wL_psiU;
	double xU_from_wU_psiU;

	if (tile->traversal_type(0) == 1){
		if (r_min*sin(rgn_from_psi_lower) + r_min < 1)
			xL_from_wL_psiL = -T_INFINITY;
		else
			xL_from_wL_psiL = rgn_from_w_lower - r_min*cos(rgn_from_psi_lower) + sqrt(pow(r_min,2) - pow(r_min*sin(rgn_from_psi_lower) - 1.0,2)); // Clockwise arc

		if (r_min - r_min*sin(rgn_from_psi_lower) < 1){
			xU_from_wL_psiL = T_INFINITY;
			btaU_from_wL_psiL = PI/2.0;
		}else{
			xU_from_wL_psiL = rgn_from_w_lower + r_min*cos(rgn_from_psi_lower) - sqrt(pow(r_min,2) - pow(r_min*sin(rgn_from_psi_lower) + 1.0,2)); // Counter-clockwise arc
			btaU_from_wL_psiL = asin(sin(rgn_from_psi_lower)+1/r_min);
		}

		if (r_min*sin(rgn_from_psi_upper) + r_min < 1){
			xL_from_wU_psiU = -T_INFINITY;
			btaL_from_wL_psiU = -PI/2.0;
		}else{
			xL_from_wU_psiU = rgn_from_w_upper - r_min*cos(rgn_from_psi_upper) + sqrt(pow(r_min,2) - pow(r_min*sin(rgn_from_psi_upper) - 1.0,2)); //Clockwise arc
			btaL_from_wL_psiU = asin(sin(rgn_from_psi_upper) - 1/r_min);
		}

		if (r_min - r_min*sin(rgn_from_psi_upper) < 1)
			xU_from_wU_psiU = T_INFINITY;
		else
			xU_from_wU_psiU = rgn_from_w_upper + r_min*cos(rgn_from_psi_upper) - sqrt(pow(r_min,2) - pow(r_min*sin(rgn_from_psi_upper) + 1.0,2));
	}else{
		xL_from_wL_psiL = r_min*sin(rgn_from_psi_lower) + sqrt(pow(r_min,2) - pow(r_min*cos(rgn_from_psi_lower) - rgn_from_w_lower,2));
		btaL_from_wL_psiU = asin(cos(rgn_from_psi_upper) - rgn_from_w_lower/r_min);

		xL_from_wU_psiU = r_min*sin(rgn_from_psi_upper) + sqrt(pow(r_min,2) - pow(r_min*cos(rgn_from_psi_upper) - rgn_from_w_upper,2));

		if (rgn_from_psi_lower < -acos((r_min - rgn_from_w_lower)/r_min)){
			xU_from_wL_psiL = r_min*sin(-rgn_from_psi_lower) - sqrt(pow(r_min,2) - pow(r_min*cos(rgn_from_psi_lower) + rgn_from_w_lower,2));
			btaU_from_wL_psiL = asin(cos(rgn_from_psi_lower) + rgn_from_w_lower/r_min);
		}else{
			xU_from_wL_psiL = T_INFINITY;
			btaU_from_wL_psiL = PI/2.0;
		}

		if (rgn_from_psi_upper < -acos((r_min - rgn_from_w_upper)/r_min)){
			xU_from_wU_psiU = r_min*sin(-rgn_from_psi_upper) - sqrt(pow(r_min,2) - pow(r_min*cos(rgn_from_psi_upper) + rgn_from_w_upper,2));
		}else
			xU_from_wU_psiU = T_INFINITY;
	}
//	std::cout << "CBRA: past if-else geometry calculations" << std::endl;
	// TODO: Check if above results are real numbers (not complex)
	// Should never happen, but find a c++ way to check to emulate MATLAB code
	RowVectorXi region_neighbors_full = RowVectorXi::LinSpaced(N_REGION_TOTAL+1,0,N_REGION_TOTAL); // +1 in MATLAB cause out of range errors
	RowVectorXi region_neighbors_p1 = region_neighbors_full.array() + 1;
	RowVectorXi is_region_neighbors = RowVectorXi::Zero(N_REGION_TOTAL+1); // vector of 'falses'

	// ------- Intersect reachable region with Q
	int is_flipped_abt_h_1 = (tile->cell_xform.row(0).array()==3).any();
	int is_flipped_abt_v_1 = (tile->cell_xform.row(0).array()==4).any();
	int is_flipped_1 = (is_flipped_abt_h_1 && !is_flipped_abt_v_1)
	                || (!is_flipped_abt_h_1 && is_flipped_abt_v_1);

	int is_flipped_2;
	if (H > 1){
		int is_flipped_abt_h_2 = (tile->cell_xform.row(1).array()==3).any();
		int is_flipped_abt_v_2 = (tile->cell_xform.row(1).array()==4).any();
	    is_flipped_2 = (is_flipped_abt_h_2 && !is_flipped_abt_v_2)
			        || (!is_flipped_abt_h_2 && is_flipped_abt_v_2);
	}

	// Crap InnerIterator not valid for Dense Matrix, only sparse, but used to be?
	std::vector<int> region_neighbors_p1_vec(region_neighbors_p1.data(),region_neighbors_p1.data() + region_neighbors_p1.size());
	for (auto idx_r_to_p1 : region_neighbors_p1_vec){
//		std::cout << "In da loop, idx = " << idx_r_to_p1 <<  std::endl;
		// --- If this region has no reachable intersection with Q, do nothing
		// The target sets of Q are already in coordinates of axes
		// attached to the second cell, so no need to flip anything here
		if(!region_target_2(idx_r_to_p1 - 1)){
			continue;
		}
		double rgn_to_w_lower_xform, rgn_to_w_upper_xform, rgn_to_psi_lower_xform, rgn_to_psi_upper_xform;
		if ((H > 1) && ((is_flipped_1 && !is_flipped_2) || (!is_flipped_1 && is_flipped_2))){
			rgn_to_w_lower_xform = 1 - REGION_BD.region_w_upper[idx_r_to_p1 - 1];
			rgn_to_w_upper_xform = 1 - REGION_BD.region_w_lower[idx_r_to_p1 - 1];
			rgn_to_psi_lower_xform = -REGION_BD.region_psi_upper[idx_r_to_p1 - 1];
			rgn_to_psi_upper_xform = -REGION_BD.region_psi_lower[idx_r_to_p1 - 1];
		}else{
			rgn_to_w_lower_xform   = REGION_BD.region_w_lower[idx_r_to_p1 - 1];
			rgn_to_w_upper_xform   = REGION_BD.region_w_upper[idx_r_to_p1 - 1];
			rgn_to_psi_lower_xform = REGION_BD.region_psi_lower[idx_r_to_p1 - 1];
			rgn_to_psi_upper_xform = REGION_BD.region_psi_upper[idx_r_to_p1 - 1];

		}

		if (xL_from_wU_psiU > rgn_to_w_upper_xform) continue;
		if (xU_from_wL_psiL < rgn_to_w_lower_xform) continue;
		if (xU_from_wU_psiU < rgn_to_w_lower_xform) continue;
		if (xL_from_wL_psiL > rgn_to_w_upper_xform) continue;

		if (btaL_from_wL_psiU > rgn_to_psi_upper_xform) continue;
		if (btaU_from_wL_psiL < rgn_to_psi_lower_xform) continue;

		is_region_neighbors(idx_r_to_p1 - 1) = 1;
	}
//	std::cout << "past the is_region_neighbors Loop" << std::endl;
	std::vector<int> indices;
	for (int idx_nbr = 0; idx_nbr < region_neighbors_full.size(); idx_nbr++){
		if (is_region_neighbors(idx_nbr)){
			indices.push_back(region_neighbors_full(idx_nbr));
		}
	}
	region_neighbors = RowVectorXi::Map(indices.data(),indices.size());
//	std::cout << "Exit CBRA" << std::endl;

}
