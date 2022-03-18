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
#include "igl/cotmatrix.h"


void getMeshVertices(std::vector<Vertex>& V);

void getNeighborFaceEdges(const MFnMesh &selectedObject, const MIntArray &connected_faceIDs, const MFloatPointArray &vertexPositions, MatrixXd& edgeMatrix);

void getEdgeWeight(const MFnMesh& selectedObject, const MIntArray& connected_faceIDs, const MFloatPointArray& vertexPositions, MatrixXd& edgeMatrix);
