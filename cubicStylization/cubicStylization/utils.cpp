#include "utils.h"

void precompute(std::vector<Vertex>& Vi, globalData& data, double cubeness) {
	// Get vertex positions as an array of points
	MFloatPointArray vertPositionsList;
	data.selectedObject.getPoints(vertPositionsList, MSpace::kWorld);
	data.vertexPositions = vertPositionsList;

	Vi.resize(vertPositionsList.length());

	// Get vertex positions as matrix
	MatrixXd vertPositions(vertPositionsList.length(), 3);
	for (int v = 0; v < vertPositionsList.length(); ++v) {
		vertPositions(v, 0) = vertPositionsList[v][0];
		vertPositions(v, 1) = vertPositionsList[v][1];
		vertPositions(v, 2) = vertPositionsList[v][2];
	}	

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

	// Pinned vertex
	data.pinnedVertexPosition.resize(1, 3);
	data.pinnedVertexPosition = vertPositions.row(0);

	data.pinnedVertexIndex.resize(1);
	data.pinnedVertexIndex(0) = 0;

	// LOCAL STEP PRECOMPUTATION
	// Get lambda * area term for all vertices
	igl::massmatrix(vertPositions, facePositions, igl::MASSMATRIX_TYPE_BARYCENTRIC, data.areaMatrix);
	
	// Get cotangent weights for weight matrix
	igl::cotmatrix_entries(vertPositions, facePositions, data.cotanW);

	// Cube normals
	data.cubeNormals = { MFloatPoint(1, 0, 0), MFloatPoint(-1, 0, 0),
											 MFloatPoint(0, 1, 0), MFloatPoint(0, -1, 0),
											 MFloatPoint(0, 0, 1), MFloatPoint(0, 0, -1) };
	for (int cn = 0; cn < 6; ++cn) {
		VectorXd normalVec(4);
		normalVec << data.cubeNormals[cn][0], data.cubeNormals[cn][1], data.cubeNormals[cn][2], 0.f;
		VectorXd transformVec = data.transformMat * normalVec;
		transformVec.normalize();
		data.cubeNormals[cn] = MFloatPoint(transformVec[0], transformVec[1], transformVec[2]);
	}

	igl::parallel_for(
		vertPositionsList.length(),
		[&Vi, &vertPositionsList, &vertPositions, &data, cubeness](const int i)
	{
		MIntArray connected_faces_unsorted;
		MItMeshVertex vertexIter(data.node);
		int lastIndex;
		vertexIter.setIndex(i, lastIndex);

		// Initialize a new vertex
		VectorXd vertexPosition(3);
		vertexPosition << vertPositionsList[i][0], vertPositionsList[i][1], vertPositionsList[i][2];
		Vertex v(i, vertexPosition);

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
		getNeighborFaceEdgesAndWeights(connected_faces, data, v);

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
		
		// Make Vertex object and store in Vertex array
		v.Ek_p = v.Ek;
		Vi[i] = v;

	}, 5);

	// GLOBAL STEP PRECOMPUTATION
	getGlobalMatrices(vertPositions, Vi, facePositions, data.Q, data.K);
	igl::min_quad_with_fixed_precompute(data.Q, data.pinnedVertexIndex, SparseMatrix<double>(), false, data.solver_data);
}

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

		MFloatPoint vert1 = data.vertexPositions[connected_vertices[0]];
		MFloatPoint vert2 = data.vertexPositions[connected_vertices[1]];
		MFloatPoint vert3 = data.vertexPositions[connected_vertices[2]];
		
		VectorXd edge1 { {vert2.x - vert1.x, vert2.y - vert1.y, vert2.z - vert1.z} };
		VectorXd edge2 { {vert3.x - vert2.x, vert3.y - vert2.y, vert3.z - vert2.z} };
		VectorXd edge3 { {vert1.x - vert3.x, vert1.y - vert3.y, vert1.z - vert3.z} };

		// Add to edge matrix and edge index matrix
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
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 2);
		weightRow++;
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 0);
		weightRow++;
		vert.W.coeffRef(weightRow, weightRow) = data.cotanW.coeffRef(connected_faceIDs[i], 1);
		weightRow++;
	}
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
	Q = 3.0 * Q;

	// K
	int numVerts = Vd.size();
	K.resize(9 * numVerts, 3 * numVerts);

	std::vector<Triplet<double>> KIJV;
	KIJV.reserve(27 * numVerts * numVerts);
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
}