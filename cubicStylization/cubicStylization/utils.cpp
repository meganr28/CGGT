#include "utils.h"

void getMeshVertices(std::vector<Vertex>& V) {
	std::unordered_map<int, std::vector<int>> faceToVertices;

	MDagPath node;
	MObject component;
	MSelectionList list;
	MFnDagNode nodeFn;
	MGlobal::getActiveSelectionList(list);
	if (list.length() == 0) {
		// no mesh selected
		return;
	}
	list.getDagPath(0, node, component);
	nodeFn.setObject(node);
	MGlobal::displayInfo(nodeFn.name().asChar());
	MFnMesh selectedObject(node);
	MFloatPointArray vertexPositions;
	selectedObject.getPoints(vertexPositions, MSpace::kWorld);

	MItMeshVertex vertexIter(node);
	MIntArray connected_faces;
	for (int i = 0; i < vertexPositions.length(); ++i) {
		int index = vertexIter.index();
		vertexIter.getConnectedFaces(connected_faces);
		MatrixXd neighborEdges(3,3 * connected_faces.length());
		getNeighborFaceEdges(selectedObject, connected_faces, vertexPositions, neighborEdges);

		std::stringstream ss;
		ss << neighborEdges;
		MGlobal::displayInfo(("Vertex: (" + std::to_string(vertexPositions[i].x) + "," + std::to_string(vertexPositions[i].y) + "," + std::to_string(vertexPositions[i].z) + ") \n" + ss.str()).c_str());
		vertexIter.next();

	}
}

void getNeighborFaceEdges(const MFnMesh& selectedObject, const MIntArray& connected_faceIDs, const MFloatPointArray& vertexPositions, MatrixXd& edgeMatrix) {
	int matindex = 0;
	for (int i = 0; i < connected_faceIDs.length(); ++i) {
		MIntArray connected_vertices;
		selectedObject.getPolygonVertices(connected_faceIDs[i], connected_vertices);

		MFloatPoint vert1 = vertexPositions[connected_vertices[0]];
		MFloatPoint vert2 = vertexPositions[connected_vertices[1]];
		MFloatPoint vert3 = vertexPositions[connected_vertices[2]];

		VectorXd edge1 { {vert1.x - vert2.x, vert1.y - vert2.y, vert1.z - vert2.z} };
		VectorXd edge2 { {vert2.x - vert3.x, vert2.y - vert3.y, vert2.z - vert3.z} };
		VectorXd edge3 { {vert3.x - vert1.x, vert3.y - vert1.y, vert3.z - vert1.z} };


		edgeMatrix(0, matindex) = edge1[0];
		edgeMatrix(1, matindex) = edge1[1];
		edgeMatrix(2, matindex) = edge1[2];
		matindex++;

		edgeMatrix(0, matindex) = edge2[0];
		edgeMatrix(1, matindex) = edge2[1];
		edgeMatrix(2, matindex) = edge2[2];
		matindex++;

		edgeMatrix(0, matindex) = edge3[0];
		edgeMatrix(1, matindex) = edge3[1];
		edgeMatrix(2, matindex) = edge3[2];
		matindex++;


		//igl::cotmatrix(vertexPositions)


	}
	
}

void getEdgeWeight(const MFnMesh& selectedObject, const MIntArray& connected_faceIDs, const MFloatPointArray& vertexPositions, MatrixXd& edgeMatrix) {

}