#pragma once

#include <maya/MPxCommand.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnMesh.h>
#include <maya/MSelectionList.h>
#include <maya/MFloatPointArray.h>
#include <maya/MItMeshVertex.h>
#include "Vertex.h"
#include "cubicstylization.h"
#include "utils.h"

class CGGTCmd : public MPxCommand
{
public:
    CGGTCmd();
    virtual ~CGGTCmd();
    static void* creator() { return new CGGTCmd(); }
    MStatus doIt(const MArgList& args);
    static MSyntax newSyntax();
};