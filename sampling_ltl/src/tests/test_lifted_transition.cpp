// test_lifted_transition.cpp
// Ben Cooper
// 11/22/2016

# include "trans_sys/hcost/hcost_interface.h"

using namespace std;
using namespace srcl;
using namespace Eigen;
using namespace HCost;

int main(int argc, char** argv )
{
	//1. Grid/Graph generation
	std::shared_ptr<SquareGrid> grid_cbta;
	grid_cbta = std::make_shared<SquareGrid>(12,12,1);

	//2. CBTA - Preprocessing
	std::map<unsigned int,std::shared_ptr<Hlevel>> Hlevels = HCost::hcost_preprocessing();

	//3. Generate input parameters for get_lifted_transition
	int H = 5;
	// Create VCell, a [rows*cols,4] matrix with VCell(i,:) = [xpos,ypos,dx,dy]
	//	int grid_rows = grid_cbta->row_size_;
	MatrixXi VCell(grid_cbta->row_size_*grid_cbta->col_size_,4);
	int total_rows_cbta = 0;
	int cell_size_cbta = grid_cbta->cell_size_;
	for (int m2 = 0; m2 < grid_cbta->row_size_; m2++)
		for (int m1 = 0; m1 < grid_cbta->col_size_; m1++){
			VCell.row(total_rows_cbta) << m1*cell_size_cbta, m2*cell_size_cbta, cell_size_cbta, cell_size_cbta;
			total_rows_cbta++;
		}
	// Verify VCell created
	std::cout << "VCell" << endl;
	std::cout << VCell << endl;

	// vertSeq conversion
	std::vector<int> vertSeq = {1, 2, 3, 4, 5, 6, 7};
    MatrixXi tile_vertices(vertSeq.size(),4);
	for (int ii = 0; ii < vertSeq.size(); ii++){
		tile_vertices.row(ii) << VCell.row(vertSeq[ii]);
	}

	std::vector<double> zta0 = {0.45,0.0};
	std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0)};

	int N_REGION_W = 25;
	int N_REGION_PSI = 25;
	int N_REGION_SPD = 1;
	int N_REGION_TOTAL = N_REGION_W*N_REGION_PSI*N_REGION_SPD - 1;

	//4. Get cost, update state (rgn_idx_init/rgn_idx_next)
	double new_cost = HCost::get_lifted_transition(H, tile_vertices, rgn_idx_init, Hlevels, N_REGION_TOTAL);

}
