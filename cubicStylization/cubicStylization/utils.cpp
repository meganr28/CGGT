#include "utils.h"

//void precompute(MFnMesh& selectedObject, MDagPath& node, std::vector<Vertex>& Vi, SparseMatrix<double>& Q, SparseMatrix<double>& K, MatrixXd& bc, VectorXi& b, igl::min_quad_with_fixed_data<double>& solver_data, double cubeness) {
void precompute(std::vector<Vertex>& Vi, globalData& data, double cubeness) {
	auto time_start = std::chrono::high_resolution_clock::now();
	std::unordered_map<int, std::vector<int>> faceToVertices;

	// Get vertex positions as an array of points
	MFloatPointArray vertPositionsList;
	data.selectedObject.getPoints(vertPositionsList, MSpace::kWorld);
	data.vertexPositions = vertPositionsList;



	Vi.resize(vertPositionsList.length());

	// Get vertex positions and face indices as matrices
	MatrixXd vertPositions(vertPositionsList.length(), 3);
	for (int v = 0; v < vertPositionsList.length(); ++v) {
		vertPositions(v, 0) = vertPositionsList[v][0];
		vertPositions(v, 1) = vertPositionsList[v][1];
		vertPositions(v, 2) = vertPositionsList[v][2];
	}

	// Pinned vertex
	data.pinnedVertexPosition.resize(1, 3);
	data.pinnedVertexPosition = vertPositions.row(0);

	data.pinnedVertexIndex.resize(1);
	data.pinnedVertexIndex(0) = 0;

	

	// Get face matrix
	int numPolygons = data.selectedObject.numPolygons();
	MatrixXi facePositions(numPolygons, 3);
	for (int f = 0; f < numPolygons; ++f) {
		MIntArray faceVerts;
		data.selectedObject.getPolygonVertices(f, faceVerts);

		facePositions(f, 0) = faceVerts[0];
		facePositions(f, 1) = faceVerts[1];
		facePositions(f, 2) = faceVerts[2];
	}

	// LOCAL STEP PRECOMPUTATION
	// Get lambda * area term for all vertices
	// SparseMatrix<double> areaMatrix;
	
	igl::massmatrix(vertPositions, facePositions, igl::MASSMATRIX_TYPE_BARYCENTRIC, data.areaMatrix);
	auto time_massmatrix = std::chrono::high_resolution_clock::now();
	auto ms_massmatrix = std::chrono::duration_cast<std::chrono::milliseconds>(time_massmatrix - time_start);
	MGlobal::displayInfo(("Finished massmatrix - time (ms): " + std::to_string(ms_massmatrix.count()) + " \n").c_str());
	// Get cotangent weights for weight matrix
	//MatrixXd cotanW;
	igl::cotmatrix_entries(vertPositions, facePositions, data.cotanW);
	auto time_cotmatrixentries = std::chrono::high_resolution_clock::now();
	auto ms_cotmatrixentries = std::chrono::duration_cast<std::chrono::milliseconds>(time_cotmatrixentries - time_massmatrix);
	MGlobal::displayInfo(("Finished cotmatrixentries - time (ms): " + std::to_string(ms_cotmatrixentries.count()) + " \n").c_str());

	// Cube normals
	data.cubeNormals = { MFloatPoint(1, 0, 0), MFloatPoint(-1, 0, 0),
											 MFloatPoint(0, 1, 0), MFloatPoint(0, -1, 0),
											 MFloatPoint(0, 0, 1), MFloatPoint(0, 0, -1) };

	//MItMeshVertex vertexIter(data.node);
	
	//data.K.resize(9 * vertPositionsList.length(), 3 * vertPositionsList.length());
	//int numVerts = vertPositionsList.length();
	//data.K.resize(9 * numVerts, 3 * numVerts);
	//std::vector<Triplet<double>> KIJV;
	//KIJV.reserve(numVerts * 18 * 3 * 4);
	igl::parallel_for(
		vertPositionsList.length(),
		[&Vi, &vertPositionsList, &vertPositions, &data, cubeness](const int i)
	{
	//for (int i = 0; i < vertPositionsList.length(); ++i) {
		//int index = vertexIter.index();
		MIntArray connected_faces_unsorted;
		MItMeshVertex vertexIter(data.node);
		int lastIndex;
		vertexIter.setIndex(i, lastIndex);
		// Initialize a new vertex
		VectorXd vertexPosition(3);
		vertexPosition << vertPositionsList[i][0], vertPositionsList[i][1], vertPositionsList[i][2];
		Vertex v(i, vertexPosition);

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
		//MatrixXd neighborEdges(3,3 * connected_faces.length());
		//MatrixXi neighborEdgeIndices(3 * connected_faces.length(), 2);
		//SparseMatrix<double> weightMatrix(3 * connected_faces.length(), 3 * connected_faces.length());
		//getNeighborFaceEdgesAndWeights(selectedObject, connected_faces, vertPositionsList, neighborEdges, neighborEdgeIndices, cotanW, weightMatrix);
		
		getNeighborFaceEdgesAndWeights(connected_faces, data, v);
		//MGlobal::displayInfo(("donewithfunction:" + std::to_string(1)).c_str());
		
		/*std::stringstream stream;
		stream << connected_faces;
		MGlobal::displayInfo(("Vertex: \n" + std::to_string(i) + "ConnectedFaceIDs: \n" + stream.str()).c_str());*/

		// Get vertex normal
		MVector vertNormal;
		data.selectedObject.getVertexNormal(i, false, vertNormal, MSpace::kWorld);
		VectorXd vertexNormal(3);
		vertexNormal << vertNormal[0], vertNormal[1], vertNormal[2];
		v.nk = vertexNormal;


		// Get lambda * area
		VectorXd areaDiagonal = data.areaMatrix.diagonal();
		v.lambda_a = cubeness * areaDiagonal(i);


		// Get target (snapped) normal
		MFloatPoint snapNormal;
		getSnappedNormal(vertNormal, data.cubeNormals, snapNormal);
		VectorXd snappedNormal(3);
		snappedNormal << snapNormal[0], snapNormal[1], snapNormal[2];
		v.tk = snappedNormal;


		/*MatrixXi E_i = v.Ei;
		SparseMatrix<double> W = v.W;
		int numEdges = E_i.rows();

		// Loop over each of the spokes and rims
		for (int j = 0; j < numEdges; ++j) {

			int ep0 = E_i(j, 0);
			int ep1 = E_i(j, 1);
			double wij = W.coeffRef(j, j);

			for (int dimSum = 0; dimSum < 3; ++dimSum) {

				// Calculate constants
				double valIJ = wij * (vertPositions(ep0, dimSum) - vertPositions(ep1, dimSum));
				double valJI = wij * (vertPositions(ep1, dimSum) - vertPositions(ep0, dimSum));

				// Set elements in matrix
				KIJV.push_back(Triplet<double>(dimSum + 9 * i, ep0, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i, ep1, valJI));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 3, ep0 + numVerts, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 3, ep1 + numVerts, valJI));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 6, ep0 + 2 * numVerts, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 6, ep1 + 2 * numVerts, valJI));
			}
		}*/
		
		// Make Vertex object and store in Vertex array
		//VectorXd vertexPosition(3);
		//vertexPosition << vertPositionsList[i][0], vertPositionsList[i][1], vertPositionsList[i][2];
		//Vertex v = Vertex(i, vertexPosition, neighborEdges, neighborEdges, neighborEdgeIndices, cotanW, weightMatrix, snappedNormal, vertexNormal, lambdaA);
		v.Ek_p = v.Ek;
		Vi[i] = v;

		/*std::stringstream ss;
		ss << neighborEdges;
		MGlobal::displayInfo(("Vertex: (" + std::to_string(vertexPositions[i].x) + "," + std::to_string(vertexPositions[i].y) + "," + std::to_string(vertexPositions[i].z) + ") \n" + ss.str()).c_str());*/
		//vertexIter.next();
		
	//}
	
	}, 5);
	auto time_loop = std::chrono::high_resolution_clock::now();
	auto ms_loop = std::chrono::duration_cast<std::chrono::milliseconds>(time_loop - time_cotmatrixentries);
	MGlobal::displayInfo(("Finished loop - time (ms): " + std::to_string(ms_loop.count()) + " \n").c_str());

	//data.K.setFromTriplets(KIJV.begin(), KIJV.end());
	// GLOBAL STEP PRECOMPUTATION
	/*data.K.setFromTriplets(KIJV.begin(), KIJV.end());
	auto time_K = std::chrono::high_resolution_clock::now();
	auto ms_K = std::chrono::duration_cast<std::chrono::milliseconds>(time_K - time_loop);
	MGlobal::displayInfo(("Finished K - time (ms): " + std::to_string(ms_K.count()) + " \n").c_str());*/



	
	//igl::cotmatrix(vertPositions, facePositions, data.Q);
	//data.Q = 3.0 * data.Q;
	getGlobalMatrices(vertPositions, Vi, facePositions, data.Q, data.K);
	auto time_globalmats = std::chrono::high_resolution_clock::now();
	auto ms_globalmats = std::chrono::duration_cast<std::chrono::milliseconds>(time_globalmats - time_loop);
	MGlobal::displayInfo(("Finished Q and SetFromTriplets (ms): " + std::to_string(ms_globalmats.count()) + " \n").c_str());
	igl::min_quad_with_fixed_precompute(data.Q, data.pinnedVertexIndex, SparseMatrix<double>(), false, data.solver_data);
	auto time_minquad = std::chrono::high_resolution_clock::now();
	auto ms_minquad = std::chrono::duration_cast<std::chrono::milliseconds>(time_minquad - time_globalmats);
	MGlobal::displayInfo(("Finished minquad - time (ms): " + std::to_string(ms_minquad.count()) + " \n").c_str());
	//std::stringstream qss;
	//qss << Q;
	//MGlobal::displayInfo(("Q: \n" + qss.str()).c_str());

	//std::stringstream qss1;
	//qss1 << K;
	//MGlobal::displayInfo(("K: \n" + qss1.str()).c_str());

	//MGlobal::displayInfo(("number of vertices: \n" + std::to_string(Vi.size())).c_str());

}

//void getNeighborFaceEdgesAndWeights(const MFnMesh& selectedObject, const MIntArray& connected_faceIDs, const MFloatPointArray& vertexPositions, MatrixXd& edgeMatrix, MatrixXi& edgeIndicesMatrix, MatrixXd& cotanW, SparseMatrix<double>& weightMatrix) {
void getNeighborFaceEdgesAndWeights(const MIntArray& connected_faceIDs, globalData& data, Vertex& vert) {
	vert.Ek.resize(3, 3 * connected_faceIDs.length());
	vert.Ei.resize(3 * connected_faceIDs.length(), 2);
	vert.W.resize(3 * connected_faceIDs.length(), 3 * connected_faceIDs.length());

	MatrixXd vertPositions(connected_faceIDs.length() + 1, 3);
	MatrixXi meshFaces(connected_faceIDs.length(), 3);
	std::unordered_map<int, int> globalToLocalIdx;

	int matindex = 0;
	int vertPosRow = 0;
	int weightRow = 0;
	
	for (int i = 0; i < connected_faceIDs.length(); ++i) {
		MIntArray connected_vertices;
		data.selectedObject.getPolygonVertices(connected_faceIDs[i], connected_vertices);

		//std::stringstream ssd;
		//ssd << connected_faceIDs[i];
		//MGlobal::displayInfo(("connected_faceIDs[i]: \n" + ssd.str()).c_str());
		//MGlobal::displayInfo(("vertexPositions size: " + std::to_string(data.vertexPositions.length()) + " \n").c_str());

		MFloatPoint vert1 = data.vertexPositions[connected_vertices[0]];
		MFloatPoint vert2 = data.vertexPositions[connected_vertices[1]];
		MFloatPoint vert3 = data.vertexPositions[connected_vertices[2]];
		
		// Insert vertex indices into global to local map
		for (int j = 0; j < 3; ++j) {
			MFloatPoint vert = data.vertexPositions[connected_vertices[j]];
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

		vert.Ei(matindex, 0) = connected_vertices[0];
		vert.Ei(matindex, 1) = connected_vertices[1];
		vert.Ek(0, matindex) = edge1[0];
		vert.Ek(1, matindex) = edge1[1];
		vert.Ek(2, matindex) = edge1[2];
		matindex++;

		vert.Ei(matindex, 0) = connected_vertices[1];
		vert.Ei(matindex, 1) = connected_vertices[2];
		vert.Ek(0, matindex) = edge2[0];
		vert.Ek(1, matindex) = edge2[1];
		vert.Ek(2, matindex) = edge2[2];
		matindex++;

		vert.Ei(matindex, 0) = connected_vertices[2];
		vert.Ei(matindex, 1) = connected_vertices[0];
		vert.Ek(0, matindex) = edge3[0];
		vert.Ek(1, matindex) = edge3[1];
		vert.Ek(2, matindex) = edge3[2];
		matindex++;

		// Add to weight matrix
		//for (int k = 0; k < 3; k++) {
			//int ki = k % 3;
			//int kj = (k + 1) % 3;
			//weightMatrix.coeffRef(weightRow, weightRow) = Q.coeffRef(connected_vertices[ki], connected_vertices[kj]);
			

			//MGlobal::displayInfo((std::to_string(connected_vertices[ki]) + " " + std::to_string(connected_vertices[kj])).c_str());
			
		//}
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 2);
		weightRow++;
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 0);
		weightRow++;
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 1);
		weightRow++;

	}

	//std::stringstream ss;
	//ss << edgeIndicesMatrix;
	//MGlobal::displayInfo(("Edge Indices Matrix: \n" + ss.str()).c_str());
	
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
	auto time_start = std::chrono::high_resolution_clock::now();
	// Q
	igl::cotmatrix(Vi, F, Q);
	Q = 3.0 * Q;
	auto time_cotmatrix = std::chrono::high_resolution_clock::now();
	auto ms_cotmatrix = std::chrono::duration_cast<std::chrono::milliseconds>(time_cotmatrix - time_start);
	MGlobal::displayInfo(("Finished Q - time (ms): " + std::to_string(ms_cotmatrix.count()) + " \n").c_str());
	/*MatrixXd cotEntries;
	igl::cotmatrix_entries(Vi, F, cotEntries);
	std::stringstream sqs;
	sqs << cotEntries;
	MGlobal::displayInfo(("cotmatrix_entries: \n" + sqs.str()).c_str());*/
	// K
	
	// Initialize to all zeroes
	int numVerts = Vd.size();
	K.resize(9 * numVerts, 3 * numVerts);
	std::vector<Triplet<double>> KIJV;
	KIJV.reserve(numVerts * 18 * 3 * 4);
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
				/*K.coeffRef(dimSum + 9 * i, ep0) += valIJ;
				K.coeffRef(dimSum + 9 * i, ep1) += valJI;
				K.coeffRef(dimSum + 9 * i + 3, ep0 + numVerts) += valIJ;
				K.coeffRef(dimSum + 9 * i + 3, ep1 + numVerts) += valJI;
				K.coeffRef(dimSum + 9 * i + 6, ep0 + 2 * numVerts) += valIJ;
				K.coeffRef(dimSum + 9 * i + 6, ep1 + 2 * numVerts) += valJI;*/
				KIJV.push_back(Triplet<double>(dimSum + 9 * i, ep0, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i, ep1, valJI));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 3, ep0 + numVerts, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 3, ep1 + numVerts, valJI));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 6, ep0 + 2 * numVerts, valIJ));
				KIJV.push_back(Triplet<double>(dimSum + 9 * i + 6, ep1 + 2 * numVerts, valJI));
			}
		}
	}
	K.setFromTriplets(KIJV.begin(), KIJV.end());
	auto time_K = std::chrono::high_resolution_clock::now();
	auto ms_K = std::chrono::duration_cast<std::chrono::milliseconds>(time_K - time_cotmatrix);
	MGlobal::displayInfo(("Finished K - time (ms): " + std::to_string(ms_K.count()) + " \n").c_str());

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