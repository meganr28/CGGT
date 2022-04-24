#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <maya/MFloatPoint.h>

using namespace Eigen;

void loadOBJ(std::string& filename, std::vector<MFloatPoint>& shape_normals);