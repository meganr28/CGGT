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
		
		MDataHandle cubenessData = data.inputValue(cubeness, &returnStatus);
		McheckErr(returnStatus, "Error getting time data handle\n");
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

	CGGTNode::cubeness = numericAttr.create("cubeness", "c",
		MFnNumericData::kDouble,
		0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating animCube time attribute\n");

	CGGTNode::coarse_faces = numericAttr.create("coarse_faces", "f",
		MFnNumericData::kDouble,
		0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating animCube time attribute\n");


	CGGTNode::geometry = typedAttr.create("geometry", "geom",
		MFnData::kMesh,
		MObject::kNullObj,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating animCube output attribute\n");
	typedAttr.setStorable(false);

	returnStatus = addAttribute(CGGTNode::cubeness);
	McheckErr(returnStatus, "ERROR adding angle attribute\n");

	returnStatus = addAttribute(CGGTNode::coarse_faces);
	McheckErr(returnStatus, "ERROR adding stepsize attribute\n");

	returnStatus = addAttribute(CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

	returnStatus = attributeAffects(CGGTNode::cubeness,
		CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	returnStatus = attributeAffects(CGGTNode::coarse_faces,
		CGGTNode::geometry);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	return MS::kSuccess;
}




