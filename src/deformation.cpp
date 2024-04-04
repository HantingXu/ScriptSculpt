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

void LetterDeform::updateLetter(std::vector<std::vector<int>>& bestDir, int stepSize)
{
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].update(bestDir[i], stepSize);
		letters[i].checkOnShape(contour);
	}
}

void LetterDeform::splitLetter()
{
	for (int i = 0; i < letters.size(); i++)
	{
		letters[i].split();
		//letters[i].checkOnShape(contour);
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
}

//only care about overlap and area variance
float LetterDeform::smoothFlowScore(const std::vector<std::vector<vec2>>& ptsPos)
{
	cv::Mat canvas = cv::Mat::zeros(shape.grayScale.size(), shape.grayScale.type());
	int tot = 0;
	for (int i = 0; i < letters.size(); i++)
	{
		int area = letters[i].getArea(canvas, ptsPos[i]);
		//std::cout << area << std::endl;
		tot += area;
	}
	int cover = cv::countNonZero(canvas);
	float overlap = tot - cover;
	cv::bitwise_and(canvas, shape.grayScale, canvas);
	float inShape = cv::countNonZero(canvas);
	float overlapTerm = overlap * letters.size() / (float)shape.area;
	float fitTerm = abs(log(shape.area / inShape));
	//std::cout << "dddddddddddddd: " << cover - inShape << std::endl;
	float illAreaTerm = (cover - inShape) / (float)shape.area;
	//std::cout << tot << "ccccccccccccccccccccccccc" << cover << std::endl;
	return overlapTerm +fitTerm * 3.f + illAreaTerm * 2.f;
}

float LetterDeform::getScore(const std::vector<std::vector<vec2>>& ptsPos)
{
	//float f = fitScore(ptsPos);
	float s = smoothFlowScore(ptsPos);
	return s;
	//std::cout << f << "ccccccccccccccccccccccccc" << s * 80.f << std::endl;
	//return fitScore(ptsPos) + smoothFlowScore(ptsPos);
	//return smoothFlowScore(ptsPos);
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

void Deform::setStep(int step)
{
	this->stepSize = step;
}

void Deform::step(std::vector<std::vector<int>>& bestDir)
{
	std::mt19937_64 rng;
	// initialize the random number generator with time-dependent seed
	uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	std::seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	// initialize a uniform distribution between 0 and 2
	std::uniform_real_distribution<double> unif(0, 3);

	std::vector<std::vector<int>> tmpDir;
	for (int i = 0; i < letterDeform->letters.size(); i++)
	{
		tmpDir.push_back(std::vector<int>(letterDeform->letters[i].controlPoints.size()));
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


void Deform::localStep(std::vector<std::vector<int>>& bestDir)
{
	std::vector<std::vector<int>> tmpDir;
	for (int i = 0; i < letterDeform->letters.size(); i++)
	{
		tmpDir.push_back(std::vector<int>(letterDeform->letters[i].controlPoints.size(), 0));
	}
	
	std::vector<std::vector<vec2>> ptsPos;
	std::array<float, 3> inoutCost;
	for (int j = 0; j < letterDeform->letters.size(); j++)
	{
		int letterLen = letterDeform->letters[j].controlPoints.size();
		//std::cout << letterDeform->letters.size() << ",,, " << letterLen << std::endl;
		for (int k = 0; k < letterLen; k++)
		{
			//std::cout << j << "," << k << std::endl;
			for (int inout = 0; inout < 3; inout++)
			{
				for (int sz = 0; sz < letterDeform->letters.size(); sz++)
				{
					ptsPos.push_back(std::vector<vec2>());
				}
				//std::cout << inout << std::endl;
				tmpDir[j][k] = inout;
				for (int i = 0; i < letterDeform->letters.size(); i++)
				{
					letterDeform->letters[i].update(tmpDir[i], ptsPos[i], stepSize);
				}
				inoutCost[inout] = letterDeform->getScore(ptsPos);
				ptsPos.clear();
			}
			auto minIt = std::min_element(inoutCost.begin(), inoutCost.end());
			size_t index = std::distance(inoutCost.begin(), minIt);
			tmpDir[j][k] = index;
		}	
	}
	bestDir = tmpDir;
}

void LetterDeform::post() {
	for (int i = 0; i < letters.size(); i++) {
		for (int j = 0; j < letters[i].controlPoints.size(); j++) {
			ControlPoint* p = letters[i].controlPoints[j].get();
			p->pos = p->pos - p->normal * 20;
		}
	}
}