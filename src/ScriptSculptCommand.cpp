#include "ScriptSculptCommand.h"
#include <maya/MGlobal.h>

ScriptSculptCmd::ScriptSculptCmd() : MPxCommand()
{
}

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
	}
	MGlobal::displayInfo("CONNECT");
	MGlobal::displayInfo(inputText.c_str());
	MGlobal::displayInfo(shapePath.c_str());
	cv::Mat previewImg(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));;
	utilityCore::genMayaImage(shapePath, inputText, fontColor, backColor, previewImg);
	std::size_t botDirPos = shapePath.find_last_of("/");
	std::string filePath = shapePath.substr(0, botDirPos) + "/" + inputText + "_font.jpg";
	cv::imwrite(filePath, previewImg);
	MGlobal::displayInfo(MString(filePath.c_str()));
	MGlobal::executeCommand("iconTextButton -e -image \"" + MString(filePath.c_str()) + "\" -width 150 -height 200 -manage 1 -visible 1 -style \"iconAndTextHorizontal\" -align \"left\" -scaleIcon previewImage;");
	return MStatus::kSuccess;
}


