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



void LetterAlignment::findSums(std::vector<std::array<Correspondence, 3>> &correspondence, int depth, float currentSum, std::vector<Correspondence>& currCor, std::vector<float> &sums, std::vector<std::vector<Correspondence>> &correspondences) {
	if (depth == shape.protrusions.size()) {
		correspondences.push_back(currCor);
		sums.push_back(currentSum);
		return;
	}
	for (int i = 0; i < 3; i++) {
		currCor[depth] = correspondence[depth][i];
		float currSum = 1 - correspondence[depth][i].locCompat * correspondence[depth][i].oriCompat;
		findSums(correspondence, depth + 1, currentSum + currSum, currCor, sums, correspondences);
	}
}


void LetterAlignment::initialAlignment() {
	std::vector<int> midPoints;
	std::vector<vec2> normals;
	utilityCore::subdivide(letters.size(), shape.centerline, midPoints, normals);

	std::vector<std::array<Correspondence, 3>> correspondence;


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
					corr[index] = Correspondence(anchor, letter, P_loc, P_sim, i);
				}
			}
		}
		correspondence.push_back(corr);
	}
	//exhausitive search
	std::vector<float> sums;
	std::vector<std::vector<Correspondence>> correspondences;
	std::vector<Correspondence> currCor(shape.protrusions.size());
	findSums(correspondence, 0, 0, currCor, sums, correspondences);
	//now need to check for incorrect order
	float minScore = INFINITY;
	auto minIt = std::min_element(sums.begin(), sums.end());
	size_t index = std::distance(sums.begin(), minIt);
	std::vector<Correspondence> bestCorrespondence = correspondences[index];
	std::vector<Protrusion*> perLetter(letters.size(), nullptr);
	std::vector<Correspondence> perLetterCor(letters.size());
	for (int i = 0; i < letters.size(); i++) {
		float bestScore = INFINITY;
		for (int j = 0; j < bestCorrespondence.size(); j++) {
			if (i == bestCorrespondence[j].letterIdx) {
				float currScore = 1 - bestCorrespondence[j].locCompat * bestCorrespondence[j].oriCompat;
				if (currScore < bestScore) {
					currScore = bestScore;
					perLetter[i] = &shape.protrusions[j];
					perLetterCor[i] = bestCorrespondence[j];
				}
			}
		}
	}
	//position letters
	std::vector<bool> processed;
	for (int i = 0; i < perLetter.size(); i++) {
		bool hasCor = false;
		if (perLetter[i] != nullptr) {
			hasCor = true;
		}
		processed.push_back(hasCor);
	}
	std::vector<int> locations;

	for (int i = 0; i < processed.size(); i++) {
		bool hasCor = processed[i];
		//float scale = shape.area / (2 * letters[i].boundingArea);
		//std::cout << letters[i].boundingArea << std::endl;
		float scale = 0.08f;
		letters[i].setScale(scale, scale);

		if (hasCor) {
			Protrusion protrusion = *perLetter[i];
			vec2 protrusionPos = vec2(shape.centerline[protrusion.projection].x, shape.centerline[protrusion.projection].y);
			vec2 p1 = vec2(protrusion.start.x, protrusion.start.y);
			vec2 p2 = vec2(protrusion.end.x, protrusion.end.y);
			vec2 protrusionCtr = (p1 + p2) / 2.f;
			vec2 anchorPos;
			for (Anchor& anchor : letters[i].anchors) {
				if (anchor.cutting.first == perLetterCor[i].anchor.cutting.first && anchor.cutting.second == perLetterCor[i].anchor.cutting.second) {
					anchorPos = scale * (anchor.cutting.first + anchor.cutting.second)/2.f;
				}
			}
			vec2 delta = protrusionCtr - anchorPos;
			letters[i].setTranslate(delta.x(), delta.y());
			//letters[i].setTranslate(protrusionPos.x(), protrusionPos.y());
			locations.push_back(protrusion.projection);
			vec2 axis = vec2(protrusion.axis[0], protrusion.axis[1]);
			axis.normalize();
			float rad;
			if (protrusion.type == ASCEND){
				rad = acosf(axis.dot(vec2(0, -1)));
			}
			else {
				rad = acosf(axis.dot(vec2(0, 1)));
			}

			letters[i].setRotate(rad * 180.f / M_PI );
		}
		else {
			locations.push_back(-1);
		}
	}

	int start = -1;
	int startIdx;
	int end = -1;
	int endIdx;
	bool isCounting = false;
	std::vector<int> indices;
	for (int i = 0; i < locations.size(); i++) {
		if (locations[i] == -1) {
			if (i == 0 || i == locations.size() - 1) {
				int loc = (i == 0) ? 0 : shape.centerline.size() - 1;
				locations[i] = loc;
				indices.push_back(i);
			}
			else {
				if (!isCounting) {
					isCounting = true;
					start = locations[i - 1];
					startIdx = i - 1;
				}
			}
		}
		else {
			if (isCounting) {
				end = locations[i];
				endIdx = i;
				int length = end - start;
				int num = endIdx - startIdx - 1;
				int space = length / (num + 1);
				for (int j = startIdx + 1; j < endIdx; j++){
					locations[j] = start + space;
					indices.push_back(j);
					space += space;
				}
				isCounting = false;
			}
		}
	}
	for (int i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		vec2 pos = vec2(shape.centerline[locations[idx]].x, shape.centerline[locations[idx]].y);
		letters[idx].setTranslate(pos.x(), pos.y());

		int index = locations[idx];
		int leftEnd1 = std::max(0, index - 3);
		int end = shape.centerline.size() - 1;
		int rightEnd1 = std::min(index + 3, end);
		int leftEnd2 = std::max(0, index - 5);
		int rightEnd2 = std::min(index + 5, end);
		Eigen::Vector2f tangent1 = Eigen::Vector2f((shape.centerline[rightEnd1] - shape.centerline[leftEnd1]).x, (shape.centerline[rightEnd1] - shape.centerline[leftEnd1]).y).normalized();
		Eigen::Vector2f tangent2 = Eigen::Vector2f((shape.centerline[rightEnd2] - shape.centerline[leftEnd2]).x, (shape.centerline[rightEnd2] - shape.centerline[leftEnd2]).y).normalized();
		Eigen::Vector2f tangent = (tangent1 + tangent2) / 2.0f;
		vec2 normal = Eigen::Vector2f(tangent[1], -tangent[0]);
		normal.normalize();
		float rad = acosf(normal.dot(vec2(0, -1)));
		letters[indices[i]].setRotate(rad * 180.f / M_PI);
	}
}


float LetterAlignment::aspectRatioScore()
{
	float totScore = 0.0f;
	for (int i = 0; i < letters.size(); i++)
	{
		totScore += log(abs(letters[i].transform.scale[1] / letters[i].transform.scale[0]));
	}
	return abs(totScore) / letters.size();
}

float LetterAlignment::fitScore()
{
	float totScore = 0.0f;
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), shape.grayScale.type());
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].getContour(canvas, false);
	}
	cv::bitwise_and(canvas, shape.grayScale, canvas);
	float B = cv::countNonZero(canvas);
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
	float areaDenom = N / (float)shape.area;
	float avgOrient = 0.0f;
	float overlap = 0.0f;
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), cv::COLOR_BGR2GRAY);

	for (int i = 0; i < N; i++)
	{
		//areas.push_back(letters[i].boundingArea * letters[i].transform.scale[0] * letters[i].transform.scale[1]);
		orientations.push_back(letters[i].transform.ori * TORADIAN);
		float area = letters[i].getContour(canvas, true);
		areas.push_back(area);
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
		varArea = pow((areas[i] - avgArea) * areaDenom, 2);
		varOrient = pow(orientations[i] - avgOrient, 2);
	}
	/*
	std::cout << "area:" << sqrtf(varArea / (float)N) << std::endl;
	std::cout <<"oriant: " << sqrtf(varOrient / (float)N) << std::endl;
	std::cout << "overlap: " << overlap << std::endl;*/
	totScore += (sqrtf(varArea / (float)N) + sqrtf(varOrient / (float)N));
	return totScore;
}


float LetterAlignment::refinedAlignment()
{
	std::cout << 0.4f * aspectRatioScore() + 0.4f * fitScore() + 1.f * smoothFlowScore() << std::endl;
	return 0.4f * aspectRatioScore() + 0.4f * fitScore() + 1.f * smoothFlowScore();
	//return smoothFlowScore();
}

void LetterAlignment::setLetters(const GASolution& sol)
{
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].transform.ori = sol.var[i * 3] * TODEGREE;
		letters[i].transform.scale = vec2(sol.var[i * 3 + 1], sol.var[i * 3 + 2]);
	}
}

void LetterAlignment::setGASolution(GASolution& sol)
{
	for (int i = 0; i < letters.size(); i++)
	{
		sol.var[i * 3] = letters[i].transform.ori * TORADIAN;
		sol.var[i * 3 + 1] = letters[i].transform.scale[0];
		sol.var[i * 3 + 2] = letters[i].transform.scale[1];
	}
}