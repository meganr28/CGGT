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

MSyntax CGGTCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(cubenessFlag, cubenessLongFlag, MSyntax::kDouble);
	return syntax;
}

MStatus CGGTCmd::doIt(const MArgList& args)
{
	// Uncomment this for testing
	// MGlobal::displayInfo("Inside doIt function");

	// Add command arguments
	MString cubenessArg = "";
	MArgDatabase argData(syntax(), args);

	// Check for command line argument
	if (argData.isFlagSet(cubenessFlag)) {
		argData.getFlagArgument(cubenessFlag, 0, cubenessArg);
		MGlobal::displayInfo("Step Size Set: " + cubenessArg);
	}
	double cubeness = cubenessArg.asDouble();

	// Call cubic stylization function
	std::vector<Vertex> V;
	cubicStylization(V, cubeness);

	return MStatus::kSuccess;
}