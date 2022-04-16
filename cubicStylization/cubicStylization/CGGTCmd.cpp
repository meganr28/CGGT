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

MSyntax CGGTCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(cubenessFlag, cubenessLongFlag, MSyntax::kDouble);
	syntax.addFlag(iterationsFlag, iterationsLongFlag, MSyntax::kDouble);
	syntax.addFlag(reductionFlag, reductionLongFlag, MSyntax::kDouble);
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

	// Call cubic stylization function
	std::vector<Vertex> V;
	cubicStylization(V, cubeness, iterations, reduction);

	return MStatus::kSuccess;
}