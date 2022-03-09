#pragma once

#include <maya/MPxCommand.h>
#include <string>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

class CGGTCmd : public MPxCommand
{
public:
    CGGTCmd();
    virtual ~CGGTCmd();
    static void* creator() { return new CGGTCmd(); }
    MStatus doIt(const MArgList& args);
    static MSyntax newSyntax();
};