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