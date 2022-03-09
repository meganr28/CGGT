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

// Syntax creator function
MSyntax CGGTCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(cubenessFlag, cubenessLongFlag, MSyntax::kDouble);
	return syntax;
}

MStatus CGGTCmd::doIt(const MArgList& args)
{
	// message in Maya output window
	cout << "Implement Me!" << endl;
	std::cout.flush();

	// message in script editor
	MGlobal::displayInfo("Implement Me!");

	// Add command arguments
	MString cubeness = "";
	MArgDatabase argData(syntax(), args);

	// Check for command line argument
	if (argData.isFlagSet(cubenessFlag)) {
		argData.getFlagArgument(cubenessFlag, 0, cubeness);
		MGlobal::displayInfo("Step Size Set: " + cubeness);
	}

	// Create sphere with radius 'cubeness'
	MGlobal::executeCommand("polySphere -r " + cubeness + "; ");

	return MStatus::kSuccess;
}