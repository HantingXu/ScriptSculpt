#include "LetterAlignment.h"

LetterAlignment::LetterAlignment(std::vector<Letter>& letters, ImgShape& shape) {
	this->letters = letters;
	this->shape = shape;
}

float LetterAlignment::positionalComp(const vec2& letter, const vec2& protrusion, int pathLength, int wordLength)
{
	vec2 diff = letter - protrusion;
	float diff2 = diff.dot(diff);
	return expf(- diff2 / powf(2.f * pathLength / wordLength, 2));
}

float LetterAlignment::orientationComp(const Anchor& anchor, const Protrusion& protrusion) {
	int anchorType = anchor.type;
	int protrusionType = protrusion.type;
	int anchorOri = anchor.orientation;
	int protusionOri = protrusion.orientation;
	if (anchorType != protrusionType) {
		return 0;
	}
	else {
		if (anchorOri == protusionOri) {
			return 1;
		}
		else if (anchorOri == STRAIGHT || anchorOri == STRAIGHT) {
			return 0.75f;
		}
		else {
			return 0.5f;
		}
	}
}

void LetterAlignment::initialAlignment() {
	//utilityCore::subdivide(const int number, const std::vector<cv::Point>&centerline, std::vector<int>&midPoints, std::vector<Eigen::Vector2f>&normals)
	//exhausitive search
	for (const Letter& letter : letters) {
		for (const Anchor& anchor : letter.anchors) {
			for (const Protrusion& protrusion : shape.protrusions) {
				//float P_loc = positionalComp(const vec2 & letter, const vec2 & protrusion, int pathLength, int wordLength);
			}
		}
	}
}

float LetterAlignment::aspectRatioScore(std::vector<Letter>& refinedLetters)
{
	float totScore = 0.0f;
	for (int i = 0; i < refinedLetters.size(); i++)
	{
		totScore += log(abs(refinedLetters[i].transform.scale[1] / refinedLetters[i].transform.scale[0]));
	}
	return totScore / refinedLetters.size();
}

float LetterAlignment::fitScore()
{
	float totScore = 0.0f;
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), cv::COLOR_BGR2GRAY);
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].getContour(canvas, false);
	}
	cv::bitwise_and(canvas, shape.grayScale, canvas);
	float B = cv::countNonZero(canvas);
	//float A = shape.area - B;
	// B-A negative
	return abs(log(shape.area / B));
}

float LetterAlignment::smoothFlowScore()
{
	int N = letters.size();
	float totScore = 0.0f;
	std::vector<float> areas;
	std::vector<float> orientations;
	float avgArea = 0.0f;
	float avgOrient = 0.0f;
	float overlap = 0.0f;
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), cv::COLOR_BGR2GRAY);

	for (int i = 0; i < N; i++)
	{
		areas.push_back(letters[i].boundingArea * letters[i].transform.scale[0] * letters[i].transform.scale[1]);
		orientations.push_back(letters[i].transform.ori);
		letters[i].getContour(canvas, false);
		overlap += areas[i];
		avgOrient += orientations[i];
	}
	avgArea = overlap;
	float cover = cv::countNonZero(canvas);
	//is it really nessisary to /shape.area?
	overlap = (overlap - cover) / (float)shape.area;
	avgArea /= (float)N;
	avgOrient /= (float)N;
	totScore += overlap;

	float varArea = 0.0f;
	float varOrient = 0.0f;
	for (int i = 0; i < N; i++)
	{
		varArea = pow(areas[i] - avgArea, 2);
		varOrient = pow(orientations[i] - avgOrient, 2);
	}
	totScore += (sqrtf(varArea / (float)N) + sqrtf(varOrient / (float)N));
	return totScore;
}

void LetterAlignment::refinedAlignment()
{

}