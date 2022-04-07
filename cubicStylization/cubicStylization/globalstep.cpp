#include "globalstep.h"

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const globalData& data) 
{
	// Solve linear system
	MatrixXd RallT = Rall.transpose();
	Map<const VectorXd> Rcol(RallT.data(), RallT.size());
	MatrixXd KT = data.K.transpose();

	MatrixXd rhs = KT * Rcol;
	
	/*igl::parallel_for(
		3,
		[&Vd, &data, &rhs](const int dim)
	{*/
	for (int dim = 0; dim < 3; dim++)
	{
		VectorXd Uc, Bc, bcc;
		Bc = rhs.block(dim * Vd.rows(), 0, Vd.rows(), 1);
		bcc = data.pinnedVertexPosition.col(dim);
		min_quad_with_fixed_solve(
			data.solver_data, Bc, bcc, VectorXd(), Uc);
		Vd.col(dim) = Uc;
	}
	//}, 1);
}