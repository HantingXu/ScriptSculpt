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
		letters[i].checkOnShape(contour);
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


void Deform::genRandSequence(int size, std::vector<int>& randSeq, std::default_random_engine& rng, bool rand)
{
	for (int i = 0; i < size; i++)
		randSeq.push_back(i);
	if(rand)
		std::shuffle(randSeq.begin(), randSeq.end(), rng);
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


void Deform::localStep(std::vector<std::vector<int>>& bestDir, bool rand)
{
	std::vector<std::vector<int>> tmpDir;
	auto rng = std::default_random_engine{};
	for (int i = 0; i < letterDeform->letters.size(); i++)
	{
		tmpDir.push_back(std::vector<int>(letterDeform->letters[i].controlPoints.size(), 0));
	}
	
	std::vector<std::vector<vec2>> ptsPos;
	std::array<float, 3> inoutCost;
	std::vector<int> randSeq;
	for (int j = 0; j < letterDeform->letters.size(); j++)
	{
		int letterLen = letterDeform->letters[j].controlPoints.size();
		genRandSequence(letterLen, randSeq, rng, rand);
		for (int k = 0; k < letterLen; k++)
		{
			int idx = randSeq[k];
			for (int inout = 0; inout < 3; inout++)
			{
				for (int sz = 0; sz < letterDeform->letters.size(); sz++)
				{
					ptsPos.push_back(std::vector<vec2>());
				}
				tmpDir[j][idx] = inout;
				for (int i = 0; i < letterDeform->letters.size(); i++)
				{
					letterDeform->letters[i].update(tmpDir[i], ptsPos[i], stepSize);
				}
				inoutCost[inout] = letterDeform->getScore(ptsPos);
				ptsPos.clear();
				
			}
			auto minIt = std::min_element(inoutCost.begin(), inoutCost.end());
			size_t index = std::distance(inoutCost.begin(), minIt);
			tmpDir[j][idx] = index;
		}
		randSeq.clear();
	}
	bestDir = tmpDir;
}

std::vector<vec2> calculateBezierPoints2(const std::vector<vec2>& controlPoints, int numPoints) {
	std::vector<vec2> curvePoints;
	for (int i = 0; i <= numPoints; ++i) {
		float t = float(i) / numPoints;
		float one_minus_t = 1.0f - t;
		vec2 point = one_minus_t * one_minus_t * one_minus_t * controlPoints[0] +
			3 * one_minus_t * one_minus_t * t * controlPoints[1] +
			3 * one_minus_t * t * t * controlPoints[2] +
			t * t * t * controlPoints[3];
		curvePoints.push_back(point);
	}
	return curvePoints;
}

void drawBezierCurve(Letter &l, std::vector<vec2> &pointss, cv::Mat& image) {
	std::cout << l.transform.ori << std::endl;
	std::cout << l.transform.scale << std::endl;
	std::cout << l.transform.pos << std::endl;
	for (int i = 0; i < pointss.size(); i += 4) {
		std::vector<vec3> controlPointsTransformed;
		for (int j = 0; j < 4; j++) {
			vec3 point = vec3(pointss[i + j].x(), -pointss[i + j].y(), 1);
			controlPointsTransformed.push_back(l.getTransformMat() * point);
		}
		std::vector<vec2> points;
		points.push_back(vec2(controlPointsTransformed[0].x(), controlPointsTransformed[0].y()));
		points.push_back(vec2(controlPointsTransformed[1].x(), controlPointsTransformed[1].y()));
		points.push_back(vec2(controlPointsTransformed[2].x(), controlPointsTransformed[2].y()));
		points.push_back(vec2(controlPointsTransformed[3].x(), controlPointsTransformed[3].y()));
		std::vector<vec2> curvePoints = calculateBezierPoints2(points, 100);
		//cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
		cv::Scalar color = cv::Scalar(255, 255, 255);
		for (size_t i = 0; i < curvePoints.size() - 1; ++i) {
			cv::line(image,
				cv::Point(curvePoints[i][0], curvePoints[i][1]),
				cv::Point(curvePoints[i + 1][0], curvePoints[i + 1][1]),
				color, 2);
		}
	}
}

void LetterDeform::post(cv::Mat& image) {
	for (int i = 0; i < letters.size(); i++) {
		if (letters[i].id == 'B') {
			std::vector<vec2> points = {
				vec2(112.9701620134295, -51.145695239910864), vec2(71.37915709306242, -51.145695239910864), vec2(47.21141099068696, -80.93384741260621), vec2(47.21141099068696, -116.34240565562142),
				vec2(47.21141099068696, -116.34240565562142), vec2(47.21141099068696, -152.8750451127006), vec2(71.37915709306242, -180.9770754643), vec2(112.9701620134295, -180.9770754643),
				vec2(112.9701620134295, -180.9770754643), vec2(154.56116693379658, -180.9770754643), vec2(178.72891303617203, -152.8750451127006), vec2(178.72891303617203, -116.34240565562142),
				vec2(178.72891303617203, -116.34240565562142), vec2(178.72891303617203, -80.93384741260621), vec2(154.56116693379658, -51.145695239910864), vec2(112.9701620134295, -51.145695239910864)};
			drawBezierCurve(letters[i], points, image);
		}
		for (int j = 0; j < letters[i].controlPoints.size(); j++) {
			ControlPoint* p = letters[i].controlPoints[j].get();
			//shrink letter boundary
			if (!p->isFixed) p->pos = p->pos - p->normal * 20;
		}
	}
}