#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MSelectionList.h>
#include <maya/MFloatPointArray.h>

#include "igl/min_quad_with_fixed.h"

using namespace Eigen;

struct globalData {

	globalData(MDagPath n) : node(n), selectedObject(n) {};

	// Vertex positions as list
	MFloatPointArray vertexPositions;

	// Selected Object
	MDagPath node;
	MFnMesh selectedObject;

	// Pinned vertex
	MatrixXd pinnedVertexPosition;
	VectorXi pinnedVertexIndex;

	// Local step
	SparseMatrix<double> areaMatrix;
	MatrixXd cotanW;

	// Global step
	igl::min_quad_with_fixed_data<double> solver_data;
	SparseMatrix<double> Q, K;

	std::vector<MFloatPoint> cubeNormals;

};