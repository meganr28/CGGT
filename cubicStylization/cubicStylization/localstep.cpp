#include "localstep.h"

void local_step(std::vector<Vertex>& Vd, MatrixXd& Rall)
{
	//int RallCounter = 0;
	igl::parallel_for(
		Vd.size(),
		[&Vd, &Rall](const int i)
	{
	
	//for (unsigned int i = 0; i < Vd.size(); ++i) {

		// creating first matrix containing E_k and n_k in the last column
		MatrixXd Ek_nk = Vd[i].Ek;
		Ek_nk.conservativeResize(Ek_nk.rows(), Ek_nk.cols() + 1);
		Ek_nk.col(Ek_nk.cols() - 1) = Vd[i].nk;

		/*std::stringstream ss2;
		ss2 << Ek_nk;
		MGlobal::displayInfo(("Ek_nk: \n" + ss2.str()).c_str());*/
		
		// creating second matrix containing W_k and lamda * a_k
		MatrixXd Wk_lam_ak = Vd[i].W;
		Wk_lam_ak.conservativeResize(Wk_lam_ak.rows() + 1, Wk_lam_ak.cols() + 1);
		//Wk_lam_ak.col(Wk_lam_ak.cols() - 1) = MatrixXd::Zero(1,Wk_lam_ak.cols());
		//Wk_lam_ak.row(Wk_lam_ak.rows() - 1) = MatrixXd::Zero(Wk_lam_ak.rows(), 1);
		for (int u = 0; u < Wk_lam_ak.cols(); ++u) {
			Wk_lam_ak(u, Wk_lam_ak.cols() - 1) = 0.0;
			Wk_lam_ak(Wk_lam_ak.rows() - 1, u) = 0.0;
		}
		Wk_lam_ak(Wk_lam_ak.rows() - 1, Wk_lam_ak.cols() - 1) = Vd[i].lambda_a;

		/*std::stringstream ssa2;
		ssa2 << Wk_lam_ak;
		MGlobal::displayInfo(("Wk_lam_ak: \n" + ssa2.str()).c_str());*/

		

		// creating third matrix containing E_kd and t_k
		MatrixXd Ekd_tk = Vd[i].Ek_p.transpose();
		Ekd_tk.conservativeResize(Ekd_tk.rows() + 1, Ekd_tk.cols());
		Ekd_tk.row(Ekd_tk.rows() - 1) = Vd[i].tk.transpose();
		
		/*std::stringstream ssas2;
		ssas2 << Ekd_tk;
		MGlobal::displayInfo(("Ekd_tk: \n" + ssas2.str()).c_str());*/
		
		// multiplying three matrices together to get X_k matrix
		MatrixXd X_k = Ek_nk * Wk_lam_ak * Ekd_tk;
		/*std::stringstream ssass2;
		ssass2 << X_k;
		MGlobal::displayInfo(("X_k: \n" + ssass2.str()).c_str());*/
		// performing SVD on X_k matrix to get 3 matrices, U_k, Sigma_k, and V_k
		JacobiSVD<MatrixXd> svd(X_k, ComputeFullU | ComputeFullV);
		
		MatrixXd V_svd_T = svd.matrixV();
		MatrixXd U_svd_T = svd.matrixU().transpose();
		Vd[i].R = V_svd_T * U_svd_T;



		/*std::stringstream ssasss2;
		ssasss2 << V_svd_T;
		MGlobal::displayInfo(("V_svd_T: \n" + ssasss2.str()).c_str());
		std::stringstream ssassds2;
		ssassds2 << U_svd_T;
		MGlobal::displayInfo(("U_svd_T: \n" + ssassds2.str()).c_str());*/


		if (Vd[i].R.determinant() < 0) {
			U_svd_T.row(3) = -U_svd_T.row(3);
			Vd[i].R = V_svd_T * U_svd_T;
		}
		/*std::stringstream ssassdsd2;
		ssassdsd2 << Vd[i].R;
		MGlobal::displayInfo(("Vd[i].R: \n" + ssassdsd2.str()).c_str());*/
		int RallIndex = i * 3;
		Rall(RallIndex, 0) = Vd[i].R(0, 0);
		Rall(RallIndex, 1) = Vd[i].R(0, 1);
		Rall(RallIndex, 2) = Vd[i].R(0, 2);
		RallIndex++;
		Rall(RallIndex, 0) = Vd[i].R(1, 0);
		Rall(RallIndex, 1) = Vd[i].R(1, 1);
		Rall(RallIndex, 2) = Vd[i].R(1, 2);
		RallIndex++;
		Rall(RallIndex, 0) = Vd[i].R(2, 0);
		Rall(RallIndex, 1) = Vd[i].R(2, 1);
		Rall(RallIndex, 2) = Vd[i].R(2, 2);
	//}
	}, 500);
	//std::stringstream ss;
	//ss << Rall;
	//MGlobal::displayInfo(("Rall: \n" + ss.str()).c_str());
}