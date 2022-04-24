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
#include <chrono>
#include <random>

#include "globaldata.h"
#include "Vertex.h"
#include "objloader.h"

#include "igl/arap_rhs.h"
#include "igl/cotmatrix.h"
#include "igl/min_quad_with_fixed.h"
#include "igl/parallel_for.h"

// Pre-compute local step matrices and global step Q and K matrices
void precompute(std::vector<Vertex>& Vi, globalData& data, double cubeness, bool randomCubeness, double random_min, double random_max, double cubenessX, double cubenessY, double cubenessZ, MString& targetOBJFilename);

// Get neigbhoring edges and weights for each edge for one vertex
void getNeighborFaceEdgesAndWeights(const MIntArray& connected_faceIDs, globalData& data, Vertex& vert);

// Snap a normal to the closest cube normal 
void getSnappedNormal(const MFloatPoint& vertexNormal, const std::vector<MFloatPoint> &cubeNormals, MFloatPoint& snappedNormal);

// Compute the Q and K matrices to solve the linear system during the global step
void getGlobalMatrices(MatrixXd& Vi, std::vector<Vertex>& Vd, MatrixXi& F, SparseMatrix<double>& Q, SparseMatrix<double>& K);
