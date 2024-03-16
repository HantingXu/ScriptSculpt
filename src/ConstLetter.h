#pragma once
#include <Eigen/Dense>
#include <vector>

using vec2 = Eigen::Vector2f;

struct Anchor {
	int type;
	int orientation;
	std::pair<int, int> cutting;
};

struct Transform {
	vec2 pos;
	vec2 ori;
	vec2 scale;
};

class Letter {
public:
	std::vector<Anchor> anchors;
	Transform transform;
	std::vector<vec2> controlPoints;
	Letter();
	Letter(char letter);
	~Letter();
};

class ConstLetters {
public:
	std::vector<Letter> letters;
	ConstLetters();
	~ConstLetters();
	const Letter& getLetter(char letter);
};