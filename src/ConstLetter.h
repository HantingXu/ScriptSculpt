#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include "sceneStruct.h"

struct Anchor {
	int type;
	int orientation;
	std::pair<vec2, vec2> cutting;
};

struct Transform {
	vec2 pos;
	float ori;
	vec2 scale;
};

class Letter {
private:
	std::vector<Anchor> anchors;
	Transform transform;
	std::vector<vec2> controlPoints;
	float boundingArea;
	mat3 getTransformMat();

public:
	Letter();
	Letter(char letter);
	~Letter();
	void generateControlPoints(char letter);
	void generateArea(char letter);
	void generateAnchorPoints(char letter);
	void drawBezierCurve(cv::Mat&);
	void drawAnchors(cv::Mat&);
	void drawControlPoints(cv::Mat&);

	void setScale(float x, float y);
	void setRotate(float angle);
	void setTranslate(float x, float y);

	void getContour();
	friend class ConstLetters;
	friend class LetterAlignment;
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