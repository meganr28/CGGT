#include "localstep.h"

void local_step(const std::vector<Vertex>& V, std::vector<Vertex>& V_d, float cubeness)
{
	// For each vertex in V:
		// Make input edge vectors(Ek)-- > store in matrix for SVD
		//	Get edge vectors'
		//	Get target normal vector(tk) with snap_normals()->pre - defined set of cube normals
		//	Get input normal vector
		//	Get lambda * a - igl function
		//	Get weights for each edge vector - igl function-- > store in matrix for SVD
	//Initialize Vertex object with above info
	//Push back to vector<Vertex>
}