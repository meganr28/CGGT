#pragma once

#include "Vertex.h"
#include <maya/MGlobal.h>
#include <vector>

void local_step(std::vector<Vertex>& Vd, MatrixXd& Rall);