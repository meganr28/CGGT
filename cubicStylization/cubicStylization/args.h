#pragma once

#include <maya/MString.h>

struct commandArgs {

	commandArgs(double c, double cX, double cY, double cZ, int it, double rp, MString &ref, MString &tobj)
		: cubeness(c), cubenessX(cX), cubenessY(cY), cubenessZ(cZ),
		randomCubeness(false), randomMin(0.01), randomMax(3.0),
		iterations(it), reductionPercent(rp), 
		referenceFrame(ref), targetObj(tobj)
	{}

	// Cubeness factors
	double cubeness;
	double cubenessX, cubenessY, cubenessZ;

	// Random cubeness
	bool randomCubeness;
	double randomMin;
	double randomMax;

	int iterations;
	double reductionPercent;
	MString referenceFrame;
	MString targetObj;

};