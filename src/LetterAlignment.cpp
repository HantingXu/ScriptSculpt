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