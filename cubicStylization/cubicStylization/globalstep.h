#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Vertex.h"
using Eigen::MatrixXd;

void global_step(MatrixXd& Vd, const MatrixXd& Rall, const MatrixXd& Q, const MatrixXd& K);