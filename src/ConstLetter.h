#pragma once
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

using vec2 = Eigen::Vector2f;

enum anchorType {ASCENDER, DESCENDER};

enum anchorOrientation {STRAIGHT, LEFTANCHOR, RIGHTANCHOR};

struct Anchor {
	anchorType type;
	anchorOrientation orientation;
	std::pair<vec2, vec2> cutting;
};

struct Transform {
	vec2 pos;
	vec2 ori;
	vec2 scale;
};

class Letter {
private:
	std::vector<Anchor> anchors;
	Transform transform;
	std::vector<vec2> controlPoints;
	float boundingArea;
public:
	Letter();
	Letter(char letter);
	~Letter();
	void generateControlPoints(char letter);
	void generateArea(char letter);
	void generateAnchorPoints(char letter);
	void drawBezierCurve(cv::Mat&);
	void drawAnchors(cv::Mat&);
	friend class ConstLetters;
};

class ConstLetters {
private:
	std::vector<Letter> letters;
public:
	ConstLetters();
	~ConstLetters();
	Letter& getLetter(char letter);
	friend class Letter;
};