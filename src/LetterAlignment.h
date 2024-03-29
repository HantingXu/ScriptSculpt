#pragma once
#include "ConstLetter.h"
#include "utilities.h"
#include <opencv2/opencv.hpp>
#include <vector>

class LetterAlignment {
private:
	std::vector<Letter> letters;
	ImgShape shape;
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
