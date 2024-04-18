#pragma once
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MImage.h>
#include "utilities.h"

class ScriptSculptCmd : public MPxCommand
{
public:
	ScriptSculptCmd();
	virtual ~ScriptSculptCmd();
	static void* creator() { return new ScriptSculptCmd(); }
	MStatus doIt(const MArgList& args);

	static MString openWinItem;
};
