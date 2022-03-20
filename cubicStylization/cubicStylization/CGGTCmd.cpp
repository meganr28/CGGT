#include "CGGTCmd.h"

#include <maya/MGlobal.h>
#include <list>

// #define TESTING_CODE

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

	// Create sphere with radius 'cubeness'
	// MGlobal::executeCommand("polySphere -r " + cubeness + "; ");

	// Call cubic stylization function
	std::vector<Vertex> V;
	cubicStylization(V, cubeness);

#ifdef TESTING_CODE
	// mapping from face id to its 3 vertex ids
	std::unordered_map<int, std::vector<int>> imTheMap;

	MDagPath node;
	MObject component;
	MSelectionList list;
	MFnDagNode nodeFn;
	MGlobal::getActiveSelectionList(list);
	for (unsigned int index = 0; index < list.length(); index++)
	{
		list.getDagPath(index, node, component);
		nodeFn.setObject(node);
		MGlobal::displayInfo(nodeFn.name().asChar());
	}
	MFnMesh selectedObject(node);
	MFloatPointArray towMater;
	selectedObject.getPoints(towMater, MSpace::kWorld);
	MFloatPointArray towMaterOUT = towMater;
	/*for (int j = 0; j < towMater.length(); ++j) {
		towMaterOUT[j].x *= j + 1;
		towMaterOUT[j].y *= j + 1;
		towMaterOUT[j].z *= j + 1;
	}*/

	/*for (int i = 0; i < towMater.length(); ++i) {
		float pointX = towMater[i].x;
		float pointY = towMater[i].y;
		float pointZ = towMater[i].z;
		MString thisPoint = (std::to_string(pointX) + " " + std::to_string(pointY) + " " + std::to_string(pointZ)).c_str();
		MGlobal::displayInfo(thisPoint);
	}*/
	selectedObject.setPoints(towMaterOUT);

	MItMeshVertex vertexIter(node);
	MIntArray connected_faces;
	for (int i = 0; i < towMater.length(); ++i) {
		int index = vertexIter.index();
		vertexIter.getConnectedFaces(connected_faces);
		for (int j = 0; j < connected_faces.length(); ++j) {
			MIntArray connected_vertices;
			selectedObject.getPolygonVertices(connected_faces[j], connected_vertices);
			std::vector<int> faceVertices = {connected_vertices[0], connected_vertices[1], connected_vertices[2] };
			MString thisPoint = ("Vertex: " + std::to_string(index) + " " + "connected Vertices: " + std::to_string(faceVertices[0]) + " " + std::to_string(faceVertices[1]) + " " + std::to_string(faceVertices[2])).c_str();
			MGlobal::displayInfo(thisPoint);
			imTheMap[connected_faces[j]] = faceVertices;
		}
		vertexIter.next();

	}
#endif

	return MStatus::kSuccess;
}