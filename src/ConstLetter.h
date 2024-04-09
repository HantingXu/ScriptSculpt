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

class ControlPoint {
private:
	bool isOutline;
	bool isFixed;
	vec2 pos;
	ControlPoint* next;
	ControlPoint* prev;
	bool color;
	vec2 normal;
public:
	ControlPoint(vec2 position, bool outline);
	~ControlPoint();
	vec2 getNormal();
	bool checkFixed(cv::Mat& contour);
	friend class Letter;
	friend class LetterAlignment;
	friend class LetterDeform;
};

class Letter {
private:
	std::vector<Anchor> anchors;
	//Transform transform;
	std::vector<sPtr<ControlPoint>> controlPoints;
	ControlPoint* start;
	float boundingArea;
	bool onProtrusion;
	int id;

public:
	Letter();
	Letter(char letter);
	~Letter();
	Transform transform;
	mat3 getTransformMat();
	void generateControlPoints(char letter);
	void generateArea(char letter);
	void generateAnchorPoints(char letter);
	void drawBezierCurve(cv::Mat&);
	void drawAnchors(cv::Mat&);
	void drawControlPoints(cv::Mat&);
	void Letter::drawNormal(cv::Mat& image);

	void setScale(float x, float y);
	void setRotate(float angle);
	void setTranslate(float x, float y);
	void setOnProtrusion(bool onProtrus);

	int getContour(cv::Mat& img, bool computeArea);
	int getArea(cv::Mat&, const std::vector<vec2>& ptsPos);
	bool getOnProtrusion();
	void split();
	void update(const std::vector<int>&, float miu);
	void update(const std::vector<int>&, std::vector<vec2>& ptsPos, float miu);
	void checkNormal();
	void checkOnShape(cv::Mat& contour);
	friend class ConstLetters;
	friend class LetterAlignment;
	friend class ControlPoint;
	friend class LetterDeform;
	friend class Deform;
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