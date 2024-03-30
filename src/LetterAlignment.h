#pragma once
#include "ConstLetter.h"
#include "utilities.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "sceneStruct.h"

struct Correspondence {
	Anchor anchor;
	Letter letter;
	float locCompat;
	float oriCompat;
	int letterIdx;

	Correspondence() : locCompat(-INFINITY), oriCompat(-INFINITY) {};
	Correspondence(const Anchor& a, const Letter& l, float loc, float ori, int idx) : anchor(a), letter(l), locCompat(loc), oriCompat(ori), letterIdx(idx) {};
};

class LetterAlignment {
private:
	ImgShape shape;
	void findSums(std::vector<std::array<Correspondence, 3>>& correspondence, int depth, float currentSum, std::vector<Correspondence>& currCor, std::vector<float>& sums, std::vector<std::vector<Correspondence>>& correspondences);
public:
	std::vector<Letter> letters;

	LetterAlignment(std::vector<Letter>& letters, ImgShape &shape);
	float positionalComp(const vec2& letter, const vec2& protrusion, int pathLength, int wordLength);
	float orientationComp(const Anchor& anchor, const Protrusion& protrusion);

	float aspectRatioScore();
	float fitScore();
	float smoothFlowScore();

	void initialAlignment();
	float refinedAlignment();

	void setLetters(const GASolution& sol);
	void setGASolution(GASolution& sol);
};
