#pragma once

#include <maya/MPxCommand.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnMesh.h>
#include <maya/MSelectionList.h>
#include <maya/MFloatPointArray.h>
#include <maya/MItMeshVertex.h>
#include <maya/MGlobal.h>
#include <list>
#include "Vertex.h"
#include "igl/arap_rhs.h"
#include "igl/cotmatrix.h"
#include "igl/min_quad_with_fixed.h"


void precompute(MFnMesh& selectedObject, MDagPath& node, std::vector<Vertex>& Vi, SparseMatrix<double>& Q, SparseMatrix<double>& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data, double cubeness);

void getNeighborFaceEdgesAndWeights(const MFnMesh &selectedObject, const MIntArray &connected_faceIDs, const MFloatPointArray &vertexPositions, MatrixXd& edgeMatrix, SparseMatrix<double>& Q, SparseMatrix<double> &weightMatrix);

void getSnappedNormal(const MFloatPoint& vertexNormal, const std::vector<MFloatPoint> &cubeNormals, MFloatPoint& snappedNormal);

void getGlobalMatrices(MatrixXd& Vi, MatrixXi& F, SparseMatrix<double>& Q, SparseMatrix<double>& K);

double getL1Norm(const MFloatPoint& vertexNormal);
