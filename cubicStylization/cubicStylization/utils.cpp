#include "utils.h"

void precompute(MFnMesh& selectedObject, MDagPath& node, std::vector<Vertex>& Vi, SparseMatrix<double>& Q, SparseMatrix<double>& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data, double cubeness) {
	std::unordered_map<int, std::vector<int>> faceToVertices;

	//MDagPath node;
	//MObject component;
	//MSelectionList list;
	//MFnDagNode nodeFn;
	//MGlobal::getActiveSelectionList(list);
	//if (list.length() == 0) {
	//	// no mesh selected
	//	return;
	//}
	//list.getDagPath(0, node, component);
	//nodeFn.setObject(node);
	//MGlobal::displayInfo(nodeFn.name().asChar());
	//MFnMesh selectedObject(node);
	MFloatPointArray vertexPositions;
	selectedObject.getPoints(vertexPositions, MSpace::kWorld);

	// Get vertex positions and face indices as matrices
	MatrixXd vertPositions(vertexPositions.length(), 3);
	for (int v = 0; v < vertexPositions.length(); ++v) {
		vertPositions(v, 0) = vertexPositions[v][0];
		vertPositions(v, 1) = vertexPositions[v][1];
		vertPositions(v, 2) = vertexPositions[v][2];
	}

	bc = vertPositions.row(0);
	b(0) = 0;


	int numPolygons = selectedObject.numPolygons();
	MatrixXi facePositions(numPolygons, 3);
	for (int f = 0; f < numPolygons; ++f) {
		MIntArray faceVerts;
		selectedObject.getPolygonVertices(f, faceVerts);

		facePositions(f, 0) = faceVerts[0];
		facePositions(f, 1) = faceVerts[1];
		facePositions(f, 2) = faceVerts[2];
	}

	//std::stringstream qss;
	//qss << Q;
	//MGlobal::displayInfo(("Q: \n" + qss.str()).c_str());

	//std::stringstream qss1;
	//qss1 << K;
	//MGlobal::displayInfo(("K: \n" + qss1.str()).c_str());

	// LOCAL STEP PRECOMPUTATION
	// Get lambda * area term for all vertices
	SparseMatrix<double> areaMatrix;
	igl::massmatrix(vertPositions, facePositions, igl::MASSMATRIX_TYPE_BARYCENTRIC, areaMatrix);

	MatrixXd cotanW;
	igl::cotmatrix_entries(vertPositions, facePositions, cotanW);

	// Cube normals
	std::vector<MFloatPoint> cubeNormals = { MFloatPoint(1, 0, 0), MFloatPoint(-1, 0, 0),
											 MFloatPoint(0, 1, 0), MFloatPoint(0, -1, 0),
											 MFloatPoint(0, 0, 1), MFloatPoint(0, 0, -1) };

	MItMeshVertex vertexIter(node);
	MIntArray connected_faces_unsorted;
	for (int i = 0; i < vertexPositions.length(); ++i) {
		int index = vertexIter.index();
		/*std::stringstream qss2;
		qss2 << vertexPositions[index];
		MGlobal::displayInfo(("vertex position: \n" + qss2.str()).c_str());*/



		// Get face edges (Nk) and weights (W)
		vertexIter.getConnectedFaces(connected_faces_unsorted);
		std::vector<int> facesToSort;
		for (int q = 0; q < connected_faces_unsorted.length(); ++q) {
			facesToSort.push_back(connected_faces_unsorted[q]);
		}
		std::sort(facesToSort.begin(), facesToSort.end());
		MIntArray connected_faces = connected_faces_unsorted;
		for (int qi = 0; qi < facesToSort.size(); ++qi) {
			connected_faces[qi] = facesToSort[qi];
		}
		std::stringstream stream;
		stream << connected_faces;
		//MGlobal::displayInfo(("Vertex: \n" + std::to_string(i) + "ConnectedFaceIDs: \n" + stream.str()).c_str());
		MatrixXd neighborEdges(3,3 * connected_faces.length());
		MatrixXi neighborEdgeIndices(3 * connected_faces.length(), 2);
		SparseMatrix<double> weightMatrix(3 * connected_faces.length(), 3 * connected_faces.length());
		getNeighborFaceEdgesAndWeights(selectedObject, connected_faces, vertexPositions, neighborEdges, neighborEdgeIndices, cotanW, weightMatrix);

		/*std::stringstream stream;
		stream << connected_faces;
		MGlobal::displayInfo(("Vertex: \n" + std::to_string(i) + "ConnectedFaceIDs: \n" + stream.str()).c_str());*/

		// Get vertex normal
		MVector vertNormal;
		selectedObject.getVertexNormal(i, false, vertNormal, MSpace::kWorld);
		VectorXd vertexNormal(3);
		vertexNormal << vertNormal[0], vertNormal[1], vertNormal[2];


		// Get lambda * area
		VectorXd areaDiagonal = areaMatrix.diagonal();
		double lambdaA = cubeness * areaDiagonal(i);


		// Get target (snapped) normal
		MFloatPoint snapNormal;
		getSnappedNormal(vertNormal, cubeNormals, snapNormal);
		VectorXd snappedNormal(3);
		snappedNormal << snapNormal[0], snapNormal[1], snapNormal[2];

		// Make Vertex object and store in Vertex array
		VectorXd vertexPosition(3);
		vertexPosition << vertexPositions[i][0], vertexPositions[i][1], vertexPositions[i][2];
		Vertex v = Vertex(i, vertexPosition, neighborEdges, neighborEdges, weightMatrix, snappedNormal, vertexNormal, lambdaA, neighborEdgeIndices);
		Vi.push_back(v);

		/*std::stringstream ss;
		ss << neighborEdges;
		MGlobal::displayInfo(("Vertex: (" + std::to_string(vertexPositions[i].x) + "," + std::to_string(vertexPositions[i].y) + "," + std::to_string(vertexPositions[i].z) + ") \n" + ss.str()).c_str());*/
		vertexIter.next();
		
	}

	// GLOBAL STEP PRECOMPUTATION
	getGlobalMatrices(vertPositions, Vi, facePositions, Q, K);

	igl::min_quad_with_fixed_precompute(Q, b, SparseMatrix<double>(), false, solver_data);

	//MGlobal::displayInfo(("number of vertices: \n" + std::to_string(Vi.size())).c_str());

}

void getNeighborFaceEdgesAndWeights(const MFnMesh& selectedObject, const MIntArray& connected_faceIDs, const MFloatPointArray& vertexPositions, MatrixXd& edgeMatrix, MatrixXi& edgeIndicesMatrix, MatrixXd& cotanW, SparseMatrix<double>& weightMatrix) {
	int matindex = 0;
	MatrixXd vertPositions(connected_faceIDs.length() + 1, 3);
	MatrixXi meshFaces(connected_faceIDs.length(), 3);
	std::unordered_map<int, int> globalToLocalIdx;

	int vertPosRow = 0;
	int weightRow = 0;
	for (int i = 0; i < connected_faceIDs.length(); ++i) {
		MIntArray connected_vertices;
		selectedObject.getPolygonVertices(connected_faceIDs[i], connected_vertices);

		//std::stringstream ssd;
		//ssd << connected_faceIDs[i];
		//MGlobal::displayInfo(("connected_faceIDs[i]: \n" + ssd.str()).c_str());

		MFloatPoint vert1 = vertexPositions[connected_vertices[0]];
		MFloatPoint vert2 = vertexPositions[connected_vertices[1]];
		MFloatPoint vert3 = vertexPositions[connected_vertices[2]];

		// Insert vertex indices into global to local map
		for (int j = 0; j < 3; ++j) {
			MFloatPoint vert = vertexPositions[connected_vertices[j]];
			std::unordered_map<int, int>::const_iterator vPosIt = globalToLocalIdx.find(connected_vertices[j]);
			if (vPosIt == globalToLocalIdx.end()) {
				globalToLocalIdx.insert(std::pair<int, int>(connected_vertices[j], vertPosRow));

				// Add to vertex position array for cotmatrix
				vertPositions(vertPosRow, 0) = vert[0];
				vertPositions(vertPosRow, 1) = vert[1];
				vertPositions(vertPosRow, 2) = vert[2];

				vertPosRow++;
			}
		}

		// Initialize face matrix for comatrix
		meshFaces(i, 0) = globalToLocalIdx.at(connected_vertices[0]);
		meshFaces(i, 1) = globalToLocalIdx.at(connected_vertices[1]);
		meshFaces(i, 2) = globalToLocalIdx.at(connected_vertices[2]);
		


		VectorXd edge1 { {vert2.x - vert1.x, vert2.y - vert1.y, vert2.z - vert1.z} };
		VectorXd edge2 { {vert3.x - vert2.x, vert3.y - vert2.y, vert3.z - vert2.z} };
		VectorXd edge3 { {vert1.x - vert3.x, vert1.y - vert3.y, vert1.z - vert3.z} };

		edgeIndicesMatrix(matindex, 0) = connected_vertices[0];
		edgeIndicesMatrix(matindex, 1) = connected_vertices[1];
		edgeMatrix(0, matindex) = edge1[0];
		edgeMatrix(1, matindex) = edge1[1];
		edgeMatrix(2, matindex) = edge1[2];
		matindex++;

		edgeIndicesMatrix(matindex, 0) = connected_vertices[1];
		edgeIndicesMatrix(matindex, 1) = connected_vertices[2];
		edgeMatrix(0, matindex) = edge2[0];
		edgeMatrix(1, matindex) = edge2[1];
		edgeMatrix(2, matindex) = edge2[2];
		matindex++;

		edgeIndicesMatrix(matindex, 0) = connected_vertices[2];
		edgeIndicesMatrix(matindex, 1) = connected_vertices[0];
		edgeMatrix(0, matindex) = edge3[0];
		edgeMatrix(1, matindex) = edge3[1];
		edgeMatrix(2, matindex) = edge3[2];
		matindex++;

		// Add to weight matrix
		//for (int k = 0; k < 3; k++) {
			//int ki = k % 3;
			//int kj = (k + 1) % 3;
			//weightMatrix.coeffRef(weightRow, weightRow) = Q.coeffRef(connected_vertices[ki], connected_vertices[kj]);
			

			//MGlobal::displayInfo((std::to_string(connected_vertices[ki]) + " " + std::to_string(connected_vertices[kj])).c_str());
			
		//}
		weightMatrix.coeffRef(weightRow, weightRow) = cotanW.coeffRef(connected_faceIDs[i], 2);
		weightRow++;
		weightMatrix.coeffRef(weightRow, weightRow) = cotanW.coeffRef(connected_faceIDs[i], 0);
		weightRow++;
		weightMatrix.coeffRef(weightRow, weightRow) = cotanW.coeffRef(connected_faceIDs[i], 1);
		weightRow++;
	}

	//std::stringstream ss;
	//ss << edgeIndicesMatrix;
	//MGlobal::displayInfo(("Edge Indices Matrix: \n" + ss.str()).c_str());
	
}

void getWeights(const MatrixXd& cotanW, MatrixXd& weightMatrix) {

}

void getSnappedNormal(const MFloatPoint& vertexNormal, const std::vector<MFloatPoint>& cubeNormals, MFloatPoint& snappedNormal) {
	int matchIdx = 0;
	double max = -INFINITY;
	for (int i = 0; i < cubeNormals.size(); ++i) {
		MFloatPoint cubeNormal = cubeNormals.at(i);
		double cosTheta = vertexNormal.x * cubeNormal.x + vertexNormal.y * cubeNormal.y + vertexNormal.z * cubeNormal.z;
		if (cosTheta > max) {
			max = cosTheta;
			matchIdx = i;
		}
	}

	snappedNormal = cubeNormals.at(matchIdx);
}

void getGlobalMatrices(MatrixXd& Vi, std::vector<Vertex>& Vd, MatrixXi& F, SparseMatrix<double>& Q, SparseMatrix<double>& K) {
	// Q
	igl::cotmatrix(Vi, F, Q);
	/*MatrixXd cotEntries;
	igl::cotmatrix_entries(Vi, F, cotEntries);
	std::stringstream sqs;
	sqs << cotEntries;
	MGlobal::displayInfo(("cotmatrix_entries: \n" + sqs.str()).c_str());*/
	// K
	
	// Initialize to all zeroes
	int numVerts = Vd.size();
	K.resize(9 * numVerts, 3 * numVerts);

	for (int i = 0; i < numVerts; ++i) {

		MatrixXi E_i = Vd[i].Ei;
		SparseMatrix<double> W = Vd[i].W;
		int numEdges = E_i.rows();

		// Loop over each of the spokes and rims
		for (int j = 0; j < numEdges; ++j) {

			int ep0 = E_i(j, 0);
			int ep1 = E_i(j, 1);
			double wij = W.coeffRef(j, j);

			for (int dimSum = 0; dimSum < 3; ++dimSum) {

				// Calculate constants
				double valIJ = wij * (Vi(ep0, dimSum) - Vi(ep1, dimSum));
				double valJI = wij * (Vi(ep1, dimSum) - Vi(ep0, dimSum));

				// Set elements in matrix
				K.coeffRef(dimSum + 9 * i, ep0) += valIJ;
				K.coeffRef(dimSum + 9 * i, ep1) += valJI;
				K.coeffRef(dimSum + 9 * i + 3, ep0 + numVerts) += valIJ;
				K.coeffRef(dimSum + 9 * i + 3, ep1 + numVerts) += valJI;
				K.coeffRef(dimSum + 9 * i + 6, ep0 + 2 * numVerts) += valIJ;
				K.coeffRef(dimSum + 9 * i + 6, ep1 + 2 * numVerts) += valJI;
			}
		}
	}

	//std::stringstream ss;
	//ss << K;
	//MGlobal::displayInfo(("K Matrix Resized: \n" + ss.str()).c_str());

	//igl::arap_rhs(Vi, F, Vi.cols(), igl::ARAP_ENERGY_TYPE_SPOKES_AND_RIMS, K);
}

double getL1Norm(const MFloatPoint& vertexNormal) {
	double absX = std::abs(vertexNormal[0]);
	double absY = std::abs(vertexNormal[1]);
	double absZ = std::abs(vertexNormal[2]);

	return absX + absY + absZ;
}