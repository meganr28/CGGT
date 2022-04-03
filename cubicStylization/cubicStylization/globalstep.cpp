#include "globalstep.h"

//void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data)
void global_step(MatrixXd& Vd, const MatrixXd& Rall, const globalData& data) 
{
	// Solve linear system
	/*std::stringstream s1s;
	s1s << Rall;
	MGlobal::displayInfo(("Rall: \n" + s1s.str()).c_str());*/
	MatrixXd RallT = Rall.transpose();
	/*std::stringstream sssa;
	sssa << RallT;
	MGlobal::displayInfo(("Rall " + std::to_string(RallT.rows()) + " " + std::to_string(RallT.cols()) + ": \n" + sssa.str()).c_str());*/
	
	/*std::stringstream asss;
	asss << newK;
	MGlobal::displayInfo(("K " + std::to_string(newK.rows()) + " " + std::to_string(newK.cols()) + ": \n" + asss.str()).c_str());*/
	
	Map<const VectorXd> Rcol(RallT.data(), RallT.size());
	//std::stringstream ssssss;
	//ssssss << Rcol;
	//MGlobal::displayInfo(("rcol " + std::to_string(Rcol.rows()) + " " + std::to_string(Rcol.cols()) + ": \n").c_str());

	MatrixXd KT = data.K.transpose();
	
	//std::stringstream sss;
	//sss << KT;
	//MGlobal::displayInfo(("KT " + std::to_string(KT.rows()) + " " + std::to_string(KT.cols()) + ": \n").c_str());
	
	
	//return;
	MatrixXd rhs = KT * Rcol;
	/*std::stringstream ss;
	ss << rhs;
	MGlobal::displayInfo(("rhs: \n" + ss.str()).c_str());*/
	
	
	
	for (int dim = 0; dim < 3; dim++)
	{
		VectorXd Uc, Bc, bcc;
		Bc = rhs.block(dim * Vd.rows(), 0, Vd.rows(), 1);
		bcc = data.pinnedVertexPosition.col(dim);
		min_quad_with_fixed_solve(
			data.solver_data, Bc, bcc, VectorXd(), Uc);
		Vd.col(dim) = Uc;
	}
	//igl::min_quad_with_fixed_solve(solver_data, rhs, bc, MatrixXd(), Vd);
	

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