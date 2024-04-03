#include "deformation.h"

LetterDeform::LetterDeform(std::vector<Letter>& letters, ImgShape& shape, cv::Mat& contr)
{
	this->letters = letters;
	this->shape = shape;
	this->contour = contr.clone();
}

void LetterDeform::updateNormal()
{
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].checkNormal();
	}
}

void LetterDeform::updateLetter(std::vector<std::vector<bool>>& bestDir, int stepSize)
{
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].update(bestDir[i], stepSize);
	}
}

float LetterDeform::fitScore(const std::vector<std::vector<vec2>>& ptsPos)
{
	float totScore = 0.0f;
	
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), shape.grayScale.type());
	
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].getArea(canvas, ptsPos[i]);
	}
	
	cv::bitwise_and(canvas, shape.grayScale, canvas);
	float B = cv::countNonZero(canvas);
	// B-A negative
	return abs(log(shape.area / B));
	return 0.0f;
}

//only care about overlap and area variance
float LetterDeform::smoothFlowScore(const std::vector<std::vector<vec2>>& ptsPos)
{
	/*
	int N = letters.size();
	float totScore = 0.0f;
	std::vector<float> areas;
	float avgArea = 0.0f;
	float areaDenom = N / (float)shape.area;
	float overlap = 0.0f;
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), cv::COLOR_BGR2GRAY);
	
	for (int i = 0; i < N; i++)
	{
		std::cout << "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk" << std::endl;
		float area = letters[i].getArea(canvas, ptsPos[i]);
		areas.push_back(area);
		overlap += areas[i];
	}
	avgArea = overlap;
	float cover = cv::countNonZero(canvas);
	//is it really nessisary to /shape.area?
	overlap = (overlap - cover) / (float)shape.area;
	avgArea /= (float)N;
	totScore += overlap;
	float varArea = 0.0f;
	float varOrient = 0.0f;
	for (int i = 0; i < N; i++)
	{
		varArea = pow((areas[i] - avgArea) * areaDenom, 2);
	}

	totScore += sqrtf(varArea / (float)N);
	return totScore;*/
	return 0.0f;
}

float LetterDeform::getScore(const std::vector<std::vector<vec2>>& ptsPos)
{
	return fitScore(ptsPos) + smoothFlowScore(ptsPos);
}

LetterDeform::~LetterDeform() {
	letters.clear();
}

Deform::Deform(const int sample,
	const int iter,
	const int step,
	const double thresh,
	LetterDeform* ltDeform)
{
	this->letterDeform = ltDeform;
	this->sampleNum = sample;
	this->maxIter = iter;
	this->stepSize = step;
	this->threshold = thresh;
}

void Deform::step(std::vector<std::vector<bool>>& bestDir)
{
	std::mt19937_64 rng;
	// initialize the random number generator with time-dependent seed
	uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	std::seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	// initialize a uniform distribution between 0 and 2
	std::uniform_real_distribution<double> unif(0, 2);

	std::vector<std::vector<bool>> tmpDir;
	for (int i = 0; i < letterDeform->letters.size(); i++)
	{
		tmpDir.push_back(std::vector<bool>(letterDeform->letters[i].controlPoints.size()));
	}
	
    float cost = FLT_MAX;
	
	for (int i = 0; i < sampleNum; i++)
	{
		std::vector<std::vector<vec2>> ptsPos;
		for (int j = 0; j < letterDeform->letters.size(); j++)
		{
			int letterLen = letterDeform->letters[j].controlPoints.size();
			for(int k = 0; k < letterLen; k++)
			{
				tmpDir[j][k] = (int)unif(rng);
			}
			ptsPos.push_back(std::vector<vec2>());
			
			letterDeform->letters[j].update(tmpDir[j], ptsPos[j], stepSize);
		}
		float singleCost = letterDeform->getScore(ptsPos);
		
		if (cost > singleCost)
		{
			//std::cout << "singleCost: " << singleCost << std::endl;
			cost = singleCost;
			bestDir = tmpDir;
		}
		ptsPos.clear();
	}
	std::cout << cost << std::endl;
}