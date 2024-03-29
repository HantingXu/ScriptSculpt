#pragma once
#include "ConstLetter.h"
#include "utilities.h"
#include <vector>

class LetterAlignment {
private:
	std::vector<Letter> letters;
	ImgShape shape;
public:
	LetterAlignment(std::vector<Letter>& letters, ImgShape &shape);
	float positionalComp(const vec3& letter, const vec3& protrusion, int pathLength, int wordLength);
	float orientationComp(const Anchor& anchor, const Protrusion& protrusion);
	void initialAlignment();
	void refinedAlignment();
};
