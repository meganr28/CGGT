#include "cubicstylization.h"

void cubicStylization(std::vector<Vertex>& Vi, commandArgs& args)
{
	// get selected mesh
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
	/*if (reduction >= 0.1) {
		MGlobal::executeCommand("select -r " + nodeFn.name() + " ;");
	}*/

	// Get transformation matrix
	MString transformMatCmd = "xform -q -ws -m";
	MDoubleArray transformMatListDouble;
	MGlobal::executeCommand(transformMatCmd, transformMatListDouble);

	MatrixXd transformMatData = MatrixXd::Identity(4,4);
	if (args.referenceFrame == "Local") {
		int tMatIndex = 0;
		for (int ti = 0; ti < 4; ++ti) {
			for (int tj = 0; tj < 4; ++tj) {
				transformMatData(tj, ti) = transformMatListDouble[tMatIndex];
				tMatIndex++;
			}
		}
	}
	std::stringstream s1s;
	s1s << transformMatData;
	MGlobal::displayInfo(("ReferenceFrameTransform: \n" + s1s.str()).c_str());

	// Triangulate, reduce, and freeze transformations
	MGlobal::executeCommand("polyTriangulate -ch 1 " + nodeFn.name() + ";");
	MGlobal::executeCommand(("polyReduce  -ver 1 -trm 0 -shp 0 -keepBorder 1 -keepMapBorder 1 -keepColorBorder 1 -keepFaceGroupBorder 1 -keepHardEdge 1 -keepCreaseEdge 1 -keepBorderWeight 0.5 -keepMapBorderWeight 0.5 -keepColorBorderWeight 0.5 -keepFaceGroupBorderWeight 0.5 -keepHardEdgeWeight 0.5 -keepCreaseEdgeWeight 0.5 -useVirtualSymmetry 0 -symmetryTolerance 0.01 -sx 0 -sy 1 -sz 0 -sw 0 -preserveTopology 1 -keepQuadsWeight 1 -vertexMapName \"\" -cachingReduce 1 -ch 1 -p " + std::to_string(args.reductionPercent) + " -vct 0 -tct 3000 -replaceOriginal 1 \"|" + nodeFn.name().asChar() + "\";").c_str());
	MGlobal::executeCommand("select -r " + nodeFn.name() + " ;");
	MGlobal::executeCommand("xform -cp;");
	MGlobal::executeCommand("makeIdentity -apply true -t 1 -r 1 -s 1 -n 0 -pn 1;");

	// Initialize global data
	globalData stylizationData(node);

	// Set transformation matrix
	//stylizationData.transformMat.resize(4, 4);
	stylizationData.transformMat = transformMatData;

	// Precomputation
	auto t1 = std::chrono::high_resolution_clock::now();
	precompute(Vi, stylizationData, args);
	auto t2 = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	MGlobal::displayInfo(("Finished precomputation - time (ms): " + std::to_string(ms_int.count()) + " \n").c_str());

	// Initialize vertex matrices and perform local-global update
	MatrixXd Vi_positions(Vi.size(), 3);
	MatrixXd V_positions(Vi.size(), 3);
	MatrixXd Vd_positions(Vi.size(), 3);

	for (unsigned int i = 0; i < Vi.size(); ++i) {
		Vi_positions(i, 0) = Vi[i].position(0);
		Vi_positions(i, 1) = Vi[i].position(1);
		Vi_positions(i, 2) = Vi[i].position(2);
		Vd_positions(i, 0) = Vi_positions(i, 0);
		Vd_positions(i, 1) = Vi_positions(i, 1);
		Vd_positions(i, 2) = Vi_positions(i, 2);
	}

	MatrixXd Rall(Vi.size() * 3, 3);
	std::vector<Vertex>& Vd = Vi;

	int maxIterations = args.iterations;
	for (int iter = 0; iter < maxIterations; ++iter) {
		// Set current vertex positions to last deformed vertex positions
		for (unsigned int i = 0; i < Vi.size(); ++i) {
			V_positions(i, 0) = Vd_positions(i, 0);
			V_positions(i, 1) = Vd_positions(i, 1);
			V_positions(i, 2) = Vd_positions(i, 2);
		}

		// local step (update R)
		local_step(Vd, Rall);

		// global step (update Vd)
		global_step(Vd_positions, Rall, stylizationData);
	}

	// Update vertices to deformed vertices
	MFloatPointArray deformedVertexPositions(Vd_positions.rows());
	for (int n = 0; n < Vd_positions.rows(); ++n) {
		//Vd_positions.row(n) = Vd_positions.row(n) + Vi[n].nk * 1.0;
		deformedVertexPositions[n].x = Vd_positions(n, 0);
		deformedVertexPositions[n].y = Vd_positions(n, 1);
		deformedVertexPositions[n].z = Vd_positions(n, 2);
	}
	selectedObject.setPoints(deformedVertexPositions);

	auto t3 = std::chrono::high_resolution_clock::now();
	auto ms_int2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1);
	MGlobal::displayInfo(("Finished cubic stylization - time (ms): " + std::to_string(ms_int2.count()) + " \n").c_str());
	MGlobal::executeCommand("xform -cp;");
	MGlobal::executeCommand("makeIdentity -apply true -t 1 -r 1 -s 1 -n 0 -pn 1;");
}