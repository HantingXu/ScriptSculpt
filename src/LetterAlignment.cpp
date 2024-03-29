#include "LetterAlignment.h"
#include <algorithm>

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
void LetterAlignment::findSums(std::map<Protrusion, std::array<Correspondence, 3>>& correspondence, int depth, float currentSum, std::map<Protrusion, Correspondence>& currCor, std::vector<float> &sums, std::vector<std::map<Protrusion, Correspondence>&> &correspondences) {
	if (depth == shape.protrusions.size()) {
		correspondences.push_back(currCor);
		sums.push_back(currentSum);
		return;
	}

	for (int i = 0; i < 3; ++i) {
		Protrusion protrusion = shape.protrusions[depth];
		currCor[protrusion] = correspondence[protrusion][i];
		float currSum = 1 - correspondence[protrusion][i].locCompat * correspondence[protrusion][i].oriCompat;
		findSums(correspondence, depth + 1, currentSum + currSum, currCor, sums, correspondences);
	}
}

void LetterAlignment::initialAlignment() {
	std::vector<int> midPoints;
	std::vector<vec2> normals;
	utilityCore::subdivide(letters.size(), shape.centerline, midPoints, normals);

	std::map<Protrusion, std::array<Correspondence, 3>> correspondence;


	//first find top 3 location compatibility for each protrusion
	for (int j = 0; j < shape.protrusions.size(); j++) {
		Protrusion protrusion = shape.protrusions[j];
		std::array<Correspondence, 3> corr;
		for (int i = 0; i < letters.size(); i++) {
			Letter letter = letters[i];
			for (const Anchor& anchor : letter.anchors) {
				vec2 anchorPos = vec2(shape.centerline[midPoints[i]].x, shape.centerline[midPoints[i]].y);
				vec2 protrusionPos = vec2(shape.centerline[protrusion.projection].x, shape.centerline[protrusion.projection].y);
				float P_loc = positionalComp(anchorPos, protrusionPos, shape.centerline.size(), letters.size());
				float P_sim = orientationComp(anchor, protrusion);
				std::array<float, 3> locCompat = { corr[0].locCompat, corr[1].locCompat, corr[2].locCompat };
				auto minIt = std::min_element(locCompat.begin(), locCompat.end());
				size_t index = std::distance(locCompat.begin(), minIt);
				if (P_loc > *minIt && P_loc > 0.1f) {
					corr[index] = Correspondence(anchor, letter, P_loc, P_sim);
				}
			}
		}
		correspondence[protrusion] = corr;
	}
	//exhausitive search
	std::vector<float> sums;
	std::vector<std::map<Protrusion, Correspondence>&> correspondences;
	std::map<Protrusion, Correspondence> currCor;
	findSums(correspondence, 0, 0, currCor, sums, correspondences);
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