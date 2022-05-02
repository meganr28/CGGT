#pragma once

#include <maya/MString.h>

struct commandArgs {

	commandArgs(double c, double cX, double cY, double cZ, int useA, int useG, int it, double rp, MString &ref, MString &gauss, MString &tobj)
		: cubeness(c), cubenessX(cX), cubenessY(cY), cubenessZ(cZ),
		usePerAxisVals(useA), useGaussMap(useG),
		randomCubeness(false), randomMin(0.01), randomMax(3.0),
		iterations(it), reductionPercent(rp), 
		referenceFrame(ref), gaussMap(gauss), targetObj(tobj)
	{}

	// Cubeness factors
	double cubeness;
	double cubenessX, cubenessY, cubenessZ;
	bool usePerAxisVals, useGaussMap;

	// Random cubeness
	bool randomCubeness;
	double randomMin;
	double randomMax;

	// Extras
	int iterations;
	double reductionPercent;
	MString referenceFrame;

	// Files
	MString gaussMap;
	MString targetObj;
};