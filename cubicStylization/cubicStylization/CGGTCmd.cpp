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
const char* targetOBJFlag = "-to", * targetOBJLongFlag = "-targetOBJ";

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
	syntax.addFlag(targetOBJFlag, targetOBJLongFlag, MSyntax::kString);
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
	MString targetOBJArg = "";
	MArgDatabase argData(syntax(), args);

	// Check for command line argument
	if (argData.isFlagSet(cubenessFlag)) {
		argData.getFlagArgument(cubenessFlag, 0, cubenessArg);
		MGlobal::displayInfo("Cubeness Factor: " + cubenessArg);
	}
	double cubeness = cubenessArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(iterationsFlag)) {
		argData.getFlagArgument(iterationsFlag, 0, iterationsArg);
		MGlobal::displayInfo("Iterations: " + iterationsArg);
	}
	double iterations = iterationsArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(reductionFlag)) {
		argData.getFlagArgument(reductionFlag, 0, reductionArg);
		MGlobal::displayInfo("Reduction: " + reductionArg);
	}
	double reduction = reductionArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(referenceFrameFlag)) {
		argData.getFlagArgument(referenceFrameFlag, 0, referenceFrameArg);
		MGlobal::displayInfo("Frame of Reference: " + referenceFrameArg);
	}
	MString reference_frame = referenceFrameArg.asChar();

	// Check for command line argument
	if (argData.isFlagSet(cubenessXFlag)) {
		argData.getFlagArgument(cubenessXFlag, 0, cubenessXArg);
		MGlobal::displayInfo("CubenessX Factor: " + cubenessXArg);
	}
	double cubenessX = cubenessXArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(cubenessYFlag)) {
		argData.getFlagArgument(cubenessYFlag, 0, cubenessYArg);
		MGlobal::displayInfo("CubenessY Factor: " + cubenessYArg);
	}
	double cubenessY = cubenessYArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(cubenessZFlag)) {
		argData.getFlagArgument(cubenessZFlag, 0, cubenessZArg);
		MGlobal::displayInfo("CubenessZ Factor: " + cubenessZArg);
	}
	double cubenessZ = cubenessZArg.asDouble();

	// Check for command line argument
	if (argData.isFlagSet(targetOBJFlag)) {
		argData.getFlagArgument(targetOBJFlag, 0, targetOBJArg);
		MGlobal::displayInfo("Target .obj filename: " + targetOBJArg);
	}
	MString targetOBJFilename = targetOBJArg.asChar();

	// Call cubic stylization function
	std::vector<Vertex> V;
	cubicStylization(V, cubeness, iterations, reduction, reference_frame, cubenessX, cubenessY, cubenessZ, targetOBJFilename);

	return MStatus::kSuccess;
}