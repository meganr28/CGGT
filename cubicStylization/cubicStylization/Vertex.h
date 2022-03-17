#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Vertex
{
public:
    Vertex(int i_d, VectorXd &pos, MatrixXd& ek, MatrixXd& ek_p, MatrixXd& w, VectorXd& t_k, VectorXd& n_k, float l_a)
        : id(i_d), position(pos),
          Ek(ek), Ek_p(ek_p), W(w), tk(t_k), nk(n_k), lambda_a(l_a)
    {}

    int id;
    VectorXd position;

    // For local step computation
    MatrixXd Ek;
    MatrixXd Ek_p;
    MatrixXd W;
    VectorXd tk;
    VectorXd nk;
    float lambda_a;
};