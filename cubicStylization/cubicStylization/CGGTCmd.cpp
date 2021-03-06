#include "CGGTCmd.h"

#include <maya/MGlobal.h>
#include <list>

CGGTCmd::CGGTCmd() : MPxCommand()
{
}

CGGTCmd::~CGGTCmd()
{
}

const char* cubenessFlag = "-c", * cubenessLongFlag = "-cubeness";
const char* iterationsFlag = "-it", * iterationsLongFlag = "-iterations";
const char* reductionFlag = "-r", * reductionLongFlag = "-reduction";
const char* referenceFrameFlag = "-f", * referenceFrameLongFlag = "-referenceFrame";
const char* cubenessXFlag = "-cx", * cubenessXLongFlag = "-cubenessX";
const char* cubenessYFlag = "-cy", * cubenessYLongFlag = "-cubenessY";
const char* cubenessZFlag = "-cz", * cubenessZLongFlag = "-cubenessZ";
const char* useAxisFlag = "-ua", * useAxisLongFlag = "-useAxis";
const char* useGaussFlag = "-ug", * useGaussLongFlag = "-useGauss";
const char* gaussMapFlag = "-gm", * gaussMapLongFlag = "-gaussMap";
const char* targetOBJFlag = "-to", * targetOBJLongFlag = "-targetOBJ";
const char* resetMeshFlag = "-re", * resetMeshLongFlag = "-resetMesh";


MSyntax CGGTCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(cubenessFlag, cubenessLongFlag, MSyntax::kDouble);
	syntax.addFlag(iterationsFlag, iterationsLongFlag, MSyntax::kDouble);
	syntax.addFlag(reductionFlag, reductionLongFlag, MSyntax::kDouble);
	syntax.addFlag(referenceFrameFlag, referenceFrameLongFlag, MSyntax::kString);
	syntax.addFlag(cubenessXFlag, cubenessXLongFlag, MSyntax::kString);
	syntax.addFlag(cubenessYFlag, cubenessYLongFlag, MSyntax::kString);
	syntax.addFlag(cubenessZFlag, cubenessZLongFlag, MSyntax::kString);
	syntax.addFlag(useAxisFlag, useAxisLongFlag, MSyntax::kString);
	syntax.addFlag(useGaussFlag, useGaussLongFlag, MSyntax::kString);
	syntax.addFlag(gaussMapFlag, gaussMapLongFlag, MSyntax::kString);
	syntax.addFlag(targetOBJFlag, targetOBJLongFlag, MSyntax::kString);
	syntax.addFlag(resetMeshFlag, resetMeshLongFlag, MSyntax::kString);
	return syntax;
}

MStatus CGGTCmd::doIt(const MArgList& args)
{
	// Uncomment this for testing
	// MGlobal::displayInfo("Inside doIt function");

	// Add command arguments
	MString cubenessArg = "";
	MString iterationsArg = "";
	MString reductionArg = "";
	MString referenceFrameArg = "";
	MString cubenessXArg = "";
	MString cubenessYArg = "";
	MString cubenessZArg = "";
	MString useAxisArg = "";
	MString useGaussArg = "";
	MString gaussMapArg = "";
	MString targetOBJArg = "";
	MString resetMeshArg = "";
	MArgDatabase argData(syntax(), args);


	// Check for cubeness argument
	if (argData.isFlagSet(cubenessFlag)) {
		argData.getFlagArgument(cubenessFlag, 0, cubenessArg);
		MGlobal::displayInfo("Cubeness Factor: " + cubenessArg);
	}
	double cubeness = cubenessArg.asDouble();

	// Check for iterations argument
	if (argData.isFlagSet(iterationsFlag)) {
		argData.getFlagArgument(iterationsFlag, 0, iterationsArg);
		MGlobal::displayInfo("Iterations: " + iterationsArg);
	}
	double iterations = iterationsArg.asDouble();

	// Check for reduction percentage argument
	if (argData.isFlagSet(reductionFlag)) {
		argData.getFlagArgument(reductionFlag, 0, reductionArg);
		MGlobal::displayInfo("Reduction: " + reductionArg);
	}
	double reduction = reductionArg.asDouble();

	// Check for frame of reference argument
	if (argData.isFlagSet(referenceFrameFlag)) {
		argData.getFlagArgument(referenceFrameFlag, 0, referenceFrameArg);
		MGlobal::displayInfo("Frame of Reference: " + referenceFrameArg);
	}
	MString reference_frame = referenceFrameArg.asChar();

	// Check for per-axis cubeness arguments
	if (argData.isFlagSet(cubenessXFlag)) {
		argData.getFlagArgument(cubenessXFlag, 0, cubenessXArg);
		MGlobal::displayInfo("CubenessX Factor: " + cubenessXArg);
	}
	double cubenessX = cubenessXArg.asDouble();

	if (argData.isFlagSet(cubenessYFlag)) {
		argData.getFlagArgument(cubenessYFlag, 0, cubenessYArg);
		MGlobal::displayInfo("CubenessY Factor: " + cubenessYArg);
	}
	double cubenessY = cubenessYArg.asDouble();

	if (argData.isFlagSet(cubenessZFlag)) {
		argData.getFlagArgument(cubenessZFlag, 0, cubenessZArg);
		MGlobal::displayInfo("CubenessZ Factor: " + cubenessZArg);
	}
	double cubenessZ = cubenessZArg.asDouble();

	// Check if using per-axis or per-vertex cubeness values
	if (argData.isFlagSet(useAxisFlag)) {
		argData.getFlagArgument(useAxisFlag, 0, useAxisArg);
		MGlobal::displayInfo("Use Axis: " + useAxisArg);
	}
	int useAxis = useAxisArg.asInt();

	if (argData.isFlagSet(useGaussFlag)) {
		argData.getFlagArgument(useGaussFlag, 0, useGaussArg);
		MGlobal::displayInfo("Use Gauss: " + useGaussArg);
	}
	int useGauss = useGaussArg.asInt();

	// Check for gauss map argument
	if (argData.isFlagSet(gaussMapFlag)) {
		argData.getFlagArgument(gaussMapFlag, 0, gaussMapArg);
		MGlobal::displayInfo("Gauss map filename: " + gaussMapArg);
	}
	MString gaussMapFilename = gaussMapArg.asChar();

	// Check for target obj argument
	if (argData.isFlagSet(targetOBJFlag)) {
		argData.getFlagArgument(targetOBJFlag, 0, targetOBJArg);
		MGlobal::displayInfo("Target .obj filename: " + targetOBJArg);
	}
	MString targetOBJFilename = targetOBJArg.asChar();

	// Check for resetMesh argument
	if (argData.isFlagSet(resetMeshFlag)) {
		argData.getFlagArgument(resetMeshFlag, 0, resetMeshArg);
		MGlobal::displayInfo("resetMesh: " + resetMeshArg);
	}
	MString resetMeshVal = resetMeshArg.asChar();

	// Initialize command line argument struct
	commandArgs cubicArgs(cubeness, cubenessX, cubenessY, cubenessZ, 
						  useAxis, useGauss, iterations, reduction, 
						  reference_frame, resetMeshVal, gaussMapFilename, targetOBJFilename);

	// Call cubic stylization function
	std::vector<Vertex> V;
	cubicStylization(V, cubicArgs);

	return MStatus::kSuccess;
}