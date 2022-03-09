#include "CGGTNode.h"

MObject     CGGTNode::cubeness;
MObject     CGGTNode::coarse_faces;
MObject     CGGTNode::geometry;
MTypeId     CGGTNode::id(0x8000);

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}

MStatus CGGTNode::compute(const MPlug& plug, MDataBlock& data) {
	MStatus returnStatus;

	if (plug == geometry) {
		// MGlobal::displayInfo("Hello World!");

		// Input handles
		MDataHandle cubenessData = data.inputValue(cubeness, &returnStatus);
		McheckErr(returnStatus, "Error getting cubeness data handle\n");
		double cubeness_data = cubenessData.asDouble();
		if (cubeness_data == 0) {
			return MS::kSuccess;
		}

		MDataHandle coarseFacesData = data.inputValue(coarse_faces, &returnStatus);
		McheckErr(returnStatus, "Error getting time data handle\n");
		double coarsefaces_data = coarseFacesData.asDouble();
		if (coarsefaces_data == 0) {
			return MS::kSuccess;
		}

		// Testing code
		MString cubenessStr = std::to_string(cubeness_data).c_str();
		MString facesStr = std::to_string(coarsefaces_data).c_str();

		MGlobal::displayInfo("Cubeness: " + cubenessStr);
		MGlobal::displayInfo("Number of faces: " + facesStr);

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

	CGGTNode::geometry = typedAttr.create("geometry", "geom",
		MFnData::kMesh,
		MObject::kNullObj,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating CGGTNode output geometry attribute\n");
	typedAttr.setStorable(false);

	// Add attributes
	returnStatus = addAttribute(CGGTNode::cubeness);
	McheckErr(returnStatus, "ERROR adding cubeness attribute\n");

	returnStatus = addAttribute(CGGTNode::coarse_faces);
	McheckErr(returnStatus, "ERROR adding coarse faces attribute\n");

	returnStatus = addAttribute(CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR adding geometry attribute\n");

	// Attribute affects
	returnStatus = attributeAffects(CGGTNode::cubeness,
		CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR in cubeness attributeAffects\n");

	returnStatus = attributeAffects(CGGTNode::coarse_faces,
		CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR in coarse faces attributeAffects\n");

	return MS::kSuccess;
}




