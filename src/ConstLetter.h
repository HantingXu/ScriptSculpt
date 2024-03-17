#pragma once
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

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
	float boundingArea;
	Letter();
	Letter(char letter);
	~Letter();
	void generateControlPoints(char letter);
	void generateArea(char letter);
	void generateAnchorPoints(char letter);
	void drawBezierCurve(cv::Mat&);
};

class ConstLetters {
public:
	std::vector<Letter> letters;
	ConstLetters();
	~ConstLetters();
	Letter& getLetter(char letter);
};