#pragma once
#include "ConstLetter.h"
#include "utilities.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct Correspondence {
	Anchor anchor;
	Letter letter;
	float locCompat;
	float oriCompat;

	Correspondence() : locCompat(-INFINITY), oriCompat(-INFINITY) {};
	Correspondence(const Anchor& a, const Letter& l, float loc, float ori) : anchor(a), letter(l), locCompat(loc), oriCompat(ori) {};
};

class LetterAlignment {
private:
	std::vector<Letter> letters;
	ImgShape shape;
	void findSums(std::map<Protrusion, std::array<Correspondence, 3>> &correspondence, int depth, float currentSum, std::map<Protrusion, Correspondence>& currCor, std::vector<float>& sums, std::vector<std::map<Protrusion, Correspondence>&> &correspondences);
public:
	LetterAlignment(std::vector<Letter>& letters, ImgShape &shape);
	float positionalComp(const vec2& letter, const vec2& protrusion, int pathLength, int wordLength);
	float orientationComp(const Anchor& anchor, const Protrusion& protrusion);

	float aspectRatioScore(std::vector<Letter>& refinedLetters);
	float fitScore();
	float smoothFlowScore();

	void initialAlignment();
	void refinedAlignment();
};
