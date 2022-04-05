#include "globalstep.h"

//void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data)
void global_step(MatrixXd& Vd, const MatrixXd& Rall, const globalData& data) 
{
	// Solve linear system
	/*std::stringstream s1s;
	s1s << Rall;
	MGlobal::displayInfo(("Rall: \n" + s1s.str()).c_str());*/
	auto time_globalstart = std::chrono::high_resolution_clock::now();
	MatrixXd RallT = Rall.transpose();
	auto time_transposer = std::chrono::high_resolution_clock::now();
	auto ms_transposer = std::chrono::duration_cast<std::chrono::milliseconds>(time_transposer - time_globalstart);
	MGlobal::displayInfo(("Finished R Transpose " + std::to_string(ms_transposer.count()) + " \n").c_str());
	/*std::stringstream sssa;
	sssa << RallT;
	MGlobal::displayInfo(("Rall " + std::to_string(RallT.rows()) + " " + std::to_string(RallT.cols()) + ": \n" + sssa.str()).c_str());*/
	
	/*std::stringstream asss;
	asss << newK;
	MGlobal::displayInfo(("K " + std::to_string(newK.rows()) + " " + std::to_string(newK.cols()) + ": \n" + asss.str()).c_str());*/
	
	Map<const VectorXd> Rcol(RallT.data(), RallT.size());
	auto time_rcol = std::chrono::high_resolution_clock::now();
	auto ms_rcol = std::chrono::duration_cast<std::chrono::milliseconds>(time_rcol - time_transposer);
	MGlobal::displayInfo(("Finished Rcol " + std::to_string(ms_rcol.count()) + " \n").c_str());
	/*std::stringstream ssss;
	ssss << Rcol;
	MGlobal::displayInfo(("Rcol : \n" + ssss.str()).c_str());*/

	MatrixXd KT = data.K.transpose();
	auto time_transposek = std::chrono::high_resolution_clock::now();
	auto ms_transposek = std::chrono::duration_cast<std::chrono::milliseconds>(time_transposek - time_rcol);
	MGlobal::displayInfo(("Finished K Transpose " + std::to_string(ms_transposek.count()) + " \n").c_str());
	
	//std::stringstream sss;
	//sss << KT;
	//MGlobal::displayInfo(("KT : \n" + sss.str()).c_str());

	//std::stringstream sass;
	//sass << data.K;
	//MGlobal::displayInfo(("K : \n" + sass.str()).c_str());

	//std::stringstream sssss;
	//sssss << data.Q;
	//MGlobal::displayInfo(("Q : \n" + sssss.str()).c_str());
	


	//return;
	MatrixXd rhs = KT * Rcol;
	auto time_rhs = std::chrono::high_resolution_clock::now();
	auto ms_rhs = std::chrono::duration_cast<std::chrono::milliseconds>(time_rhs - time_transposek);
	MGlobal::displayInfo(("Finished KT * Rcol " + std::to_string(ms_rhs.count()) + " \n").c_str());
	/*std::stringstream ss;
	ss << rhs;
	MGlobal::displayInfo(("rhs: \n" + ss.str()).c_str());*/
	
	
	//std::stringstream qss1;
	//qss1 << Vd;
	//MGlobal::displayInfo(("starting vertex positions: \n" + qss1.str()).c_str());
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
	auto time_minquadsolve = std::chrono::high_resolution_clock::now();
	auto ms_minquadsolve = std::chrono::duration_cast<std::chrono::milliseconds>(time_minquadsolve - time_rhs);
	MGlobal::displayInfo(("Finished minquadwithfixedsolve " + std::to_string(ms_minquadsolve.count()) + " \n").c_str());
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