#pragma once

#include <Eigen/Dense>
#include <vector>
#include <maya/MGlobal.h>
#include "igl/min_quad_with_fixed.h"
#include "Vertex.h"
using Eigen::MatrixXd;

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data);