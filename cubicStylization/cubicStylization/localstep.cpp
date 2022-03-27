#include "localstep.h"

void local_step(std::vector<Vertex>& Vd, MatrixXd& Rall)
{
	int RallCounter = 0;
	for (unsigned int i = 0; i < Vd.size(); ++i) {

		// creating first matrix containing E_k and n_k in the last column
		MatrixXd Ek_nk = Vd[i].Ek;
		Ek_nk.conservativeResize(Ek_nk.rows(), Ek_nk.cols() + 1);
		Ek_nk.col(Ek_nk.cols() - 1) = Vd[i].nk;

		
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

		

		// creating third matrix containing E_kd and t_k
		MatrixXd Ekd_tk = Vd[i].Ek_p.transpose();
		Ekd_tk.conservativeResize(Ekd_tk.rows() + 1, Ekd_tk.cols());
		Ekd_tk.row(Ekd_tk.rows() - 1) = Vd[i].tk.transpose();
		
		

		// multiplying three matrices together to get X_k matrix
		MatrixXd X_k = Ek_nk * Wk_lam_ak * Ekd_tk;

		// performing SVD on X_k matrix to get 3 matrices, U_k, Sigma_k, and V_k
		JacobiSVD<MatrixXd> svd(X_k, ComputeFullU | ComputeFullV);
		
		MatrixXd V_svd_T = svd.matrixV();
		MatrixXd U_svd_T = svd.matrixU().transpose();
		Vd[i].R = V_svd_T * U_svd_T;

		if (Vd[i].R.determinant() < 0) {
			U_svd_T.row(3) = -U_svd_T.row(3);
			Vd[i].R = V_svd_T * U_svd_T;
		}

		Rall(0, RallCounter) = Vd[i].R(0, 0);
		Rall(1, RallCounter) = Vd[i].R(1, 0);
		Rall(2, RallCounter) = Vd[i].R(2, 0);
		RallCounter++;
		Rall(0, RallCounter) = Vd[i].R(0, 1);
		Rall(1, RallCounter) = Vd[i].R(1, 1);
		Rall(2, RallCounter) = Vd[i].R(2, 1);
		RallCounter++;
		Rall(0, RallCounter) = Vd[i].R(0, 2);
		Rall(1, RallCounter) = Vd[i].R(1, 2);
		Rall(2, RallCounter) = Vd[i].R(2, 2);
		RallCounter++;
	}
	std::stringstream ss;
	ss << Rall;
	MGlobal::displayInfo(("Rall: \n" + ss.str()).c_str());
}