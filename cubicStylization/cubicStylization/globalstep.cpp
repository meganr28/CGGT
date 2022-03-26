#include "globalstep.h"

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K)
{
	// Solve linear system
	MatrixXd RallT = Rall.transpose();
	MatrixXd KT = K.transpose();
	MatrixXd Qinv = Q.inverse();

	Vd = Qinv * KT * RallT;
}