#include "ScriptSculptCommand.h"
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MObjectArray.h>
#include <maya/MDagModifier.h>

LetterDeform ScriptSculptCmd::letters;
bool ScriptSculptCmd::hasPreview;

ScriptSculptCmd::ScriptSculptCmd() : MPxCommand()
{}

ScriptSculptCmd::~ScriptSculptCmd()
{
}

MStatus ScriptSculptCmd::doIt(const MArgList& args)
{
	MStatus status;
	vec3 fontColor = vec3(255, 255, 255);
	vec3 backColor = vec3(0, 0, 0);
	std::string inputText;
	std::string shapePath;
	bool isPreview = false;
	for (int i = 0; i < args.length(); i++)
	{
		if (MString("-word") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			MString word = args.asString(++i, &status);
			if (MS::kSuccess == status)
			{
				inputText = word.asChar();
			}
		}
		else if (MString("-shapePath") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			MString path = args.asString(++i, &status);
			if (MS::kSuccess == status) 
			{ 
				shapePath = path.asChar();
			}
		}
		else if (MString("-bRed") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int bRed = args.asInt(++i, &status);
			if (MS::kSuccess == status) 
			{
				backColor[2] = bRed;
			}
		}
		else if (MString("-bGreen") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int bGreen = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				backColor[1] = bGreen;
			}
		}
		else if (MString("-bBlue") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int bBlue = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				backColor[0] = bBlue;
			}
		}
		else if (MString("-fRed") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int fRed = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				fontColor[2] = fRed;
			}
		}
		else if (MString("-fGreen") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int fGreen = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				fontColor[1] = fGreen;
			}
		}
		else if (MString("-fBlue") == args.asString(i, &status) &&
			MS::kSuccess == status)
		{
			int fBlue = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				fontColor[0] = fBlue;
			}
		}
		else if (MString("-p") == args.asString(i, &status) &&
			MS::kSuccess == status) 
		{
			int p = args.asInt(++i, &status);
			if (MS::kSuccess == status)
			{
				isPreview = (p == 1) ? true : false;
			}
		}
	}
	MGlobal::displayInfo("CONNECT");
	MGlobal::displayInfo(inputText.c_str());
	MGlobal::displayInfo(shapePath.c_str());
	if (isPreview) {
		cv::Mat previewImg(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));;
		utilityCore::genMayaImage(shapePath, inputText, fontColor, backColor, previewImg, ScriptSculptCmd::letters);
		ScriptSculptCmd::hasPreview = true;
		std::size_t botDirPos = shapePath.find_last_of("/");
		std::string filePath = shapePath.substr(0, botDirPos) + "/" + inputText + "_font.jpg";
		cv::imwrite(filePath, previewImg);
		MGlobal::displayInfo(MString(filePath.c_str()));
		MGlobal::executeCommand("iconTextButton -e -image \"" + MString(filePath.c_str()) + "\" -width 200 -height 200 -manage 1 -visible 1 -style \"iconAndTextHorizontal\" -align \"left\" -scaleIcon previewImage;");
	}
	else {
		if (ScriptSculptCmd::hasPreview) {
			for (Letter& l : ScriptSculptCmd::letters.letters) {
				std::vector<float> points;
				l.generatePointArray(points);
				int numPoints = points.size();
				MGlobal::displayInfo(MString() + numPoints);
				MPointArray pointArray;
				MObjectArray curveObjects;
				MDagModifier dagMod;
				for (int i = 0; i < numPoints; i += 3) {
					pointArray.append(MPoint(points[i], points[i + 1], points[i + 2]));
				}

				for (int i = 0; i < pointArray.length(); i += 4) {
					if (i + 3 < numPoints) {
						MPointArray curvePoints;
						curvePoints.append(pointArray[i]);
						curvePoints.append(pointArray[i + 1]);
						curvePoints.append(pointArray[i + 2]);
						curvePoints.append(pointArray[i + 3]);

						MDoubleArray knots;
						knots.append(0); knots.append(0); knots.append(0);
						knots.append(1); knots.append(1); knots.append(1);

						MFnNurbsCurve curveFn;
						MObject curveObj = curveFn.create(curvePoints, knots, 3, MFnNurbsCurve::kOpen, false, false, MObject::kNullObj, &status);
						if (!status) {
							MGlobal::displayError("Failed to create curve");
							return status;
						}
						curveObjects.append(curveObj);
					}
				}
				std::vector<float> innerPoints;
				l.generateInnerPointArray(innerPoints);
				int numInnerPoints = innerPoints.size();
				MPointArray innerPointArray;
				for (int i = 0; i < numInnerPoints; i += 3) {
					innerPointArray.append(MPoint(innerPoints[i], innerPoints[i + 1], innerPoints[i + 2]));
				}
				for (int i = 0; i < innerPointArray.length(); i += 4) {
					if (i + 3 < numPoints) {
						MPointArray curvePoints;
						curvePoints.append(innerPointArray[i]);
						curvePoints.append(innerPointArray[i + 1]);
						curvePoints.append(innerPointArray[i + 2]);
						curvePoints.append(innerPointArray[i + 3]);

						MDoubleArray knots;
						knots.append(0); knots.append(0); knots.append(0);
						knots.append(1); knots.append(1); knots.append(1);

						MFnNurbsCurve curveFn;
						MObject curveObj = curveFn.create(curvePoints, knots, 3, MFnNurbsCurve::kOpen, false, false, MObject::kNullObj, &status);
						if (!status) {
							MGlobal::displayError("Failed to create curve");
							return status;
						}
						curveObjects.append(curveObj);
					}
				}

				if (curveObjects.length() > 0) {
					MObject groupNode = dagMod.createNode("transform", MObject::kNullObj, &status);
					if (!status) {
						MGlobal::displayError("Failed to create group transform");
						return status;
					}

					for (unsigned int k = 0; k < curveObjects.length(); k++) {
						status = dagMod.reparentNode(curveObjects[k], groupNode);
						if (!status) {
							MGlobal::displayError("Failed to reparent curve");
							return status;
						}
					}
					dagMod.doIt();
				}
			}
			
		}
		else 
		{
			MGlobal::displayInfo("Please generate preview first.");
		}
	}
	return MStatus::kSuccess;
}


