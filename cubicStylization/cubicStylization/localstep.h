#pragma once

#include "Vertex.h"
#include <maya/MGlobal.h>
#include <vector>
#include "igl/parallel_for.h"

void local_step(std::vector<Vertex>& Vd, MatrixXd& Rall);