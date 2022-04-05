#include "cubicstylization.h"

void cubicStylization(std::vector<Vertex>& Vi, float cubeness)
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

	// Initialize global data
	globalData stylizationData(node);
	
	// Precomputation
	/*MatrixXd bc(1, 3);
	VectorXi b(1);
	igl::min_quad_with_fixed_data<double> solver_data;
	SparseMatrix<double> Q, K;
	precompute(selectedObject, node, Vi, Q, K, bc, b, solver_data, cubeness);*/
	auto t1 = std::chrono::high_resolution_clock::now();
	precompute(Vi, stylizationData, cubeness);
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

	// Stopping parameters
	double stopping_criteria = 0.001;
	//double diff = std::numeric_limits<float>::max();
	double diff = 1.f;
	int iteration = 1;

	MatrixXd Rall(Vi.size() * 3, 3);
	std::vector<Vertex>& Vd = Vi;
	auto time_mainloop = std::chrono::high_resolution_clock::now();
	auto ms_mainloop = std::chrono::duration_cast<std::chrono::milliseconds>(time_mainloop - t2);
	MGlobal::displayInfo(("Finished setting up main loop: " + std::to_string(ms_mainloop.count()) + " \n").c_str());
	while (diff > stopping_criteria) {
		//MGlobal::displayInfo(("Iteration: " + std::to_string(iteration)).c_str());

		// Set current vertex positions to last deformed vertex positions
		for (unsigned int i = 0; i < Vi.size(); ++i) {
			V_positions(i, 0) = Vd_positions(i, 0);
			V_positions(i, 1) = Vd_positions(i, 1);
			V_positions(i, 2) = Vd_positions(i, 2);
		}

		// local step (update R)
		local_step(Vd, Rall);
		auto time_localstep = std::chrono::high_resolution_clock::now();
		auto ms_localstep = std::chrono::duration_cast<std::chrono::milliseconds>(time_localstep - time_mainloop);
		MGlobal::displayInfo(("Finished local step: " + std::to_string(ms_localstep.count()) + " \n").c_str());
		// global step (update Vd)
		global_step(Vd_positions, Rall, stylizationData);
		auto time_globalstep = std::chrono::high_resolution_clock::now();
		auto ms_globalstep = std::chrono::duration_cast<std::chrono::milliseconds>(time_globalstep - time_localstep);
		MGlobal::displayInfo(("Finished global step: " + std::to_string(ms_globalstep.count()) + " \n").c_str());

		//double newToPrev = (Vd_positions - V_positions).cwiseAbs().maxCoeff();
		//double prevToOrig = (V_positions - Vi_positions).cwiseAbs().maxCoeff();
		
		//diff = newToPrev / prevToOrig;
		diff -= 1;
		iteration += 1;
	}

	//MGlobal::displayInfo("Finished iterations");
	/*std::stringstream qss1;
	qss1 << Vd_positions;
	MGlobal::displayInfo(("final vertex positions: \n" + qss1.str()).c_str());*/
	// Update vertices to deformed vertices
	MFloatPointArray deformedVertexPositions(Vd_positions.rows());
	for (int n = 0; n < Vd_positions.rows(); ++n) {
		deformedVertexPositions[n].x = Vd_positions(n, 0);
		deformedVertexPositions[n].y = Vd_positions(n, 1);
		deformedVertexPositions[n].z = Vd_positions(n, 2);
	}
	selectedObject.setPoints(deformedVertexPositions);
	auto t3 = std::chrono::high_resolution_clock::now();
	auto ms_int2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1);
	MGlobal::displayInfo(("Finished cubic stylization - time (ms): " + std::to_string(ms_int2.count()) + " \n").c_str());
}