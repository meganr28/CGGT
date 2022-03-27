#include "globalstep.h"

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data)
{
	// Solve linear system
	MatrixXd RallT = Rall.transpose();
	MatrixXd KT = K.transpose();
	MatrixXd rhs = KT * RallT;
	igl::min_quad_with_fixed_solve(solver_data, rhs, bc, MatrixXd(), Vd);

	std::stringstream ss;
	ss << Vd;
	MGlobal::displayInfo(("Vd: \n" + ss.str()).c_str());
	/*MatrixXd Qinv = Q.inverse();

	std::stringstream ss;
	ss << RallT;
	MGlobal::displayInfo(("RallT: \n" + ss.str()).c_str());

	std::stringstream ss1;
	ss1 << KT;
	MGlobal::displayInfo(("KT: " + std::to_string(KT.rows()) + " " + std::to_string(KT.cols()) + "\n" + ss1.str()).c_str());

	std::stringstream ss2;
	ss2 << Qinv;
	MGlobal::displayInfo(("Qinv: \n" + ss2.str()).c_str());
	return;*/
	//Vd = Qinv * KT * RallT;
}