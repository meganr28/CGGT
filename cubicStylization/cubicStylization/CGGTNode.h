#pragma once

#include <maya/MPxNode.h>
#include <string>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include "maya/MFnNumericAttribute.h"
#include "maya/MFnTypedAttribute.h"
#include "maya/MFnUnitAttribute.h"
#include "maya/MfnMeshData.h"
#include <maya/MGlobal.h>
#include "maya/MPointArray.h"
#include "maya/MFloatPointArray.h"
#include "maya/MFnMesh.h"

class CGGTNode : public MPxNode
{
public:
	CGGTNode() {};
	~CGGTNode() override {};
	MStatus compute(const MPlug& plug, MDataBlock& data) override;
	static  void* creator();
	static  MStatus initialize();

	static MObject cubeness;
	static MObject coarse_faces;
	static MObject geometry;
	static MTypeId id;

};