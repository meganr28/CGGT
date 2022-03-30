#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;

class Vertex
{
public:
    Vertex(int i_d, VectorXd &pos, MatrixXd& ek, MatrixXd& ek_p, SparseMatrix<double>& w, VectorXd& t_k, VectorXd& n_k, double l_a, MatrixXi& ei)
        : id(i_d), position(pos),
          Ek(ek), Ek_p(ek_p), Ei(ei), W(w), 
          tk(t_k), nk(n_k), lambda_a(l_a), R(MatrixXd())
    {}

    int id;
    VectorXd position;

    // For local step computation
    MatrixXd Ek;
    MatrixXd Ek_p;
    MatrixXi Ei;
    SparseMatrix<double> W;
    VectorXd tk;
    VectorXd nk;
    double lambda_a;

    // local step output
    MatrixXd R;
};