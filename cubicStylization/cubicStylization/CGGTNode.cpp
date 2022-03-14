#include "CGGTNode.h"

MObject     CGGTNode::cubeness;
MObject     CGGTNode::coarse_faces;
MObject     CGGTNode::input_geometry;
MObject     CGGTNode::output_geometry;
MTypeId     CGGTNode::id(0x8000);

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}

MStatus CGGTNode::compute(const MPlug& plug, MDataBlock& data) {
	MStatus returnStatus;

	if (plug == output_geometry) {
		// MGlobal::displayInfo("Hello World!");

		// Input handles
		MDataHandle cubenessData = data.inputValue(cubeness, &returnStatus);
		McheckErr(returnStatus, "Error getting cubeness data handle\n");
		double cubeness_data = cubenessData.asDouble();
		if (cubeness_data == 0) {
			return MS::kSuccess;
		}

		MDataHandle coarseFacesData = data.inputValue(coarse_faces, &returnStatus);
		McheckErr(returnStatus, "Error getting coarse faces data handle\n");
		double coarsefaces_data = coarseFacesData.asDouble();
		if (coarsefaces_data == 0) {
			return MS::kSuccess;
		}

		MDataHandle inputGeometryData = data.inputValue(input_geometry, &returnStatus);
		McheckErr(returnStatus, "Error getting input geometry data handle\n");
		MObject inputgeometry_data = inputGeometryData.asMesh();
		if (inputgeometry_data.isNull()) {
			MGlobal::displayInfo("megan reddy");
			return MS::kSuccess;
		}
		MDataHandle outputGeometryData = data.outputValue(output_geometry, &returnStatus);
		McheckErr(returnStatus, "Error getting output geometry data handle\n");
		outputGeometryData.copy(inputGeometryData);

		// Testing code
		MString cubenessStr = std::to_string(cubeness_data).c_str();
		MString facesStr = std::to_string(coarsefaces_data).c_str();

		MGlobal::displayInfo("Cubeness: " + cubenessStr);
		MGlobal::displayInfo("Number of faces: " + facesStr);

		std::unordered_map<int, std::vector<int>> imTheMap;

		/*MDagPath node;
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
		MFnMesh selectedObject(node);*/
		MFnMesh selectedObject(outputGeometryData.asMesh());
		MFloatPointArray towMater;
		selectedObject.getPoints(towMater, MSpace::kWorld);
		MGlobal::displayInfo(("vertices: " + std::to_string(towMater.length())).c_str());
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
		//selectedObject.setPoints(towMaterOUT);
		MDagPath node;
		selectedObject.getPath(node);
		MItMeshVertex vertexIter(node);
		MIntArray connected_faces;
		for (int i = 0; i < towMater.length(); ++i) {
			int index = vertexIter.index();
			vertexIter.getConnectedFaces(connected_faces);
			int faceCount = 0;
			//MGlobal::displayInfo(("faces: " + std::to_string(connected_faces.length())).c_str());
			vertexIter.numConnectedFaces(faceCount);
			MGlobal::displayInfo(("faces: " + std::to_string(faceCount)).c_str());
			for (int j = 0; j < connected_faces.length(); ++j) {
				MIntArray connected_vertices;
				selectedObject.getPolygonVertices(connected_faces[j], connected_vertices);
				std::vector<int> faceVertices = { connected_vertices[0], connected_vertices[1], connected_vertices[2] };
				MString thisPoint = ("Vertex: " + std::to_string(index) + " " + "connected Vertices: " + std::to_string(faceVertices[0]) + " " + std::to_string(faceVertices[1]) + " " + std::to_string(faceVertices[2])).c_str();
				MGlobal::displayInfo(thisPoint);
				imTheMap[connected_faces[j]] = faceVertices;
			}
			vertexIter.next();

		}


		data.setClean(plug);
	}
	else
		return MS::kUnknownParameter;

	return MS::kSuccess;
}

void* CGGTNode::creator() {
	return new CGGTNode;
}

MStatus CGGTNode::initialize() {
	MFnUnitAttribute unitAttr;
	MFnTypedAttribute typedAttr;
	MFnNumericAttribute numericAttr;
	MStatus returnStatus;

	// Create attributes
	CGGTNode::cubeness = numericAttr.create("cubeness", "c",
		MFnNumericData::kDouble,
		0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating CGGTNode cubeness attribute\n");

	CGGTNode::coarse_faces = numericAttr.create("coarse_faces", "f",
		MFnNumericData::kDouble,
		0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating CGGTNode coarse faces attribute\n");

	CGGTNode::input_geometry = typedAttr.create("input_geometry", "geom_i",
		MFnData::kMesh,
		MObject::kNullObj,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating CGGTNode input input_geometry attribute\n");

	CGGTNode::output_geometry = typedAttr.create("output_geometry", "geom_o",
		MFnData::kMesh,
		MObject::kNullObj,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating CGGTNode output output_geometry attribute\n");
	typedAttr.setStorable(false);

	// Add attributes
	returnStatus = addAttribute(CGGTNode::cubeness);
	McheckErr(returnStatus, "ERROR adding cubeness attribute\n");

	returnStatus = addAttribute(CGGTNode::coarse_faces);
	McheckErr(returnStatus, "ERROR adding coarse faces attribute\n");

	returnStatus = addAttribute(CGGTNode::input_geometry);
	McheckErr(returnStatus, "ERROR adding input_geometry attribute\n");

	returnStatus = addAttribute(CGGTNode::output_geometry);
	McheckErr(returnStatus, "ERROR adding output_geometry attribute\n");

	// Attribute affects
	returnStatus = attributeAffects(CGGTNode::cubeness,
		CGGTNode::output_geometry);
	McheckErr(returnStatus, "ERROR in cubeness attributeAffects\n");

	returnStatus = attributeAffects(CGGTNode::coarse_faces,
		CGGTNode::output_geometry);
	McheckErr(returnStatus, "ERROR in coarse faces attributeAffects\n");

	returnStatus = attributeAffects(CGGTNode::input_geometry,
		CGGTNode::output_geometry);
	McheckErr(returnStatus, "ERROR in input geometry attributeAffects\n");

	return MS::kSuccess;
}




