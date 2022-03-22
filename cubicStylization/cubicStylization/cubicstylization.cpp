#include "cubicstylization.h"

void cubicStylization(std::vector<Vertex>& Vi, float cubeness)
{
	// Precomputation
	getMeshVertices(Vi, cubeness);

	MatrixXd Vi_positions(Vi.size(), 3);
	MatrixXd V_positions(Vi.size(), 3);
	MatrixXd Vd_positions(Vi.size(), 3);

	for (unsigned int i = 0; i < Vi.size(); ++i) {
		Vi_positions(i, 0) = Vi[i].position(0);
		Vi_positions(i, 1) = Vi[i].position(1);
		Vi_positions(i, 2) = Vi[i].position(2);
	}

	double stopping_criteria = 0.001;
	double diff = std::numeric_limits<float>::max();
	int iteration = 0;
	std::vector<Vertex>& Vd = Vi;
	while (diff > stopping_criteria) {
		for (unsigned int i = 0; i < Vi.size(); ++i) {
			V_positions(i, 0) = Vd[i].position(0);
			V_positions(i, 1) = Vd[i].position(1);
			V_positions(i, 2) = Vd[i].position(2);
		}

		// local step (update R)
		local_step(Vd);
		// global step (update Vd)
		global_step(Vd);

		for (unsigned int i = 0; i < Vi.size(); ++i) {
			Vd_positions(i, 0) = Vd[i].position(0);
			Vd_positions(i, 1) = Vd[i].position(1);
			Vd_positions(i, 2) = Vd[i].position(2);
		}

		double newToPrev = (Vd_positions - V_positions).cwiseAbs().maxCoeff();
		double prevToOrig = (V_positions - Vi_positions).cwiseAbs().maxCoeff();
		
		diff = newToPrev / prevToOrig;
	}
	
	// while not converged:
		// Local step
		//	For each vertex :
		// Solve SVD
		//	Check determinant
		//	Global step
		//	Solve one linear system to find V'
		//	V = V'
		//	Check stopping criteria

	// Update vertices
}