#pragma once
#include "ConstLetter.h"
#include <random>
#include <chrono>

class LetterDeform
{
private:
	cv::Mat contour;
	ImgShape shape;
	
public:
	std::vector<Letter> letters;
	LetterDeform(std::vector<Letter>& letters, ImgShape& shape, cv::Mat& contr);
	~LetterDeform();

	void updateNormal();
	void updateLetter(std::vector<std::vector<int>>& bestDir, int stepSize);
	void splitLetter();

	float fitScore(const std::vector<std::vector<vec2>>& ptsPos);
	float smoothFlowScore(const std::vector<std::vector<vec2>>& ptsPos);
	float getScore(const std::vector<std::vector<vec2>>& ptsPos);

	void post();
};

class Deform
{
private:
	int sampleNum;
	int maxIter;
	int stepSize;
	double threshold;
	LetterDeform* letterDeform;
public:
	Deform(const int sample,
	const int iter,
	const int step,
	const double thresh,
	LetterDeform* ltDeform);

	void step(std::vector<std::vector<int>>& bestDir);
	void localStep(std::vector<std::vector<int>>& bestDir);
	void setStep(int stepSize);
	double computeCost();
	void updateCtrlPoint();
	void initRound();
	void hillClimbing();
};