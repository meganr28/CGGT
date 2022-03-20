#include "cubicstylization.h"

void cubicStylization(std::vector<Vertex>& V, float cubeness)
{
	// Precomputation
	getMeshVertices(V, cubeness);
	
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