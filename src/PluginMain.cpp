#include <maya/MPxCommand.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MSimple.h>
#include <maya/MPointArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MDGModifier.h>
#include <maya/MPlugArray.h>
#include <maya/MVector.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MStringArray.h>

#include "ScriptSculptCommand.h"

MStatus initializePlugin(MObject obj)
{
    MStatus status = MStatus::kSuccess;
    MFnPlugin plugin(obj, "MyPlugin", "1.0", "Any");
    
    // Register Command
    status = plugin.registerCommand("ScriptSculptCmd", ScriptSculptCmd::creator);
    if (!status) {
        status.perror("registerCommand");
        return status;
    }
    
    std::string p = plugin.loadPath().asChar();
    MGlobal::displayInfo(p.c_str());
    std::string pluginSource = "C:/Users/2000/ScriptSculpt/src";
    char buffer1[2048];
    std::string pluginSource1 = "source \"" + pluginSource + "/ScriptSculptGUI.mel\";";
    //std::string pp = "source C:/Users/2000/ScriptSculpt/src/ScriptSculptGUI.mel";
    sprintf_s(buffer1, 2048, pluginSource1.c_str(), plugin.loadPath());
    //sprintf_s(buffer1, 2048, pp.c_str(), plugin.loadPath());
    MGlobal::executeCommand(buffer1, true);

    const MString openWinCmd = "createWindow();";
    plugin.addMenuItem("Create ScriptSculptGUI", "mainCreateMenu", openWinCmd, "");
    
    return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus status = MStatus::kSuccess;
	MFnPlugin plugin(obj);
    
	// De-register Command
	status = plugin.deregisterCommand("ScriptSculptCmd");
	if (!status) {
		status.perror("deregisterCommand");
		return status;
	}

	MStringArray menuNames = MStringArray();
	menuNames.append("Open_ScriptSculpt_Dialog");
	plugin.removeMenuItem(menuNames);
    
	return status;
}