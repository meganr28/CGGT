#include "globalstep.h"

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const globalData& data) 
{
	// Solve linear system
	MatrixXd RallT = Rall.transpose();
	// need to stack RallT together to be able to multiply it with our K matrix transposed
	Map<VectorXd> RallTColumns(RallT.data(), RallT.size());
	MatrixXd KT = data.K.transpose();

	MatrixXd rhs = KT * RallTColumns;
	
	for (int i = 0; i < 3; ++i) {
		// grab the block in our rhs matrix (KT * Rcol) and use to solve for column of deformed vert positions
		VectorXd rhs_dim_i = rhs.block(i * Vd.rows(), 0, Vd.rows(), 1);
		VectorXd pinned_position_dim_i = data.pinnedVertexPosition.col(i);
		VectorXd vertPos_dim_i;
		// igl function to solve linear system, we used a precompute function for this in our precompute function to get the solver_data
		min_quad_with_fixed_solve(data.solver_data, rhs_dim_i, pinned_position_dim_i, VectorXd(), vertPos_dim_i);
		Vd.col(i) = vertPos_dim_i;
	}
}