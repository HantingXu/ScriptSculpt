#include "deformation.h"

LetterDeform::LetterDeform() 
{}

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
		std::vector<vec2> points;
		if (letters[i].id == 'B') {
			points = {
				vec2(112.9701620134295, -51.145695239910864), vec2(71.37915709306242, -51.145695239910864), vec2(47.21141099068696, -80.93384741260621), vec2(47.21141099068696, -116.34240565562142),
				vec2(47.21141099068696, -116.34240565562142), vec2(47.21141099068696, -152.8750451127006), vec2(71.37915709306242, -180.9770754643), vec2(112.9701620134295, -180.9770754643),
				vec2(112.9701620134295, -180.9770754643), vec2(154.56116693379658, -180.9770754643), vec2(178.72891303617203, -152.8750451127006), vec2(178.72891303617203, -116.34240565562142),
				vec2(178.72891303617203, -116.34240565562142), vec2(178.72891303617203, -80.93384741260621), vec2(154.56116693379658, -51.145695239910864), vec2(112.9701620134295, -51.145695239910864)};
			//drawBezierCurve(letters[i], points, image);
		}
		else if (letters[i].id == 'P') {
			points = {
			vec2(117.85540259612226, 147.02753195159806), vec2(74.68065115001806, 147.02753195159806), vec2(49.59261990430887, 116.10507483479371), vec2(49.59261990430887, 79.3481918468942),
			vec2(49.59261990430887, 79.3481918468942), vec2(49.59261990430887, 41.42442368477565), vec2(74.68065115001806, 12.252294329299838), vec2(117.85540259612226, 12.252294329299838),
			vec2(117.85540259612226, 12.252294329299838), vec2(161.03015404222646, 12.252294329299838), vec2(186.11818528793566, 41.42442368477565), vec2(186.11818528793566, 79.3481918468942),
			vec2(186.11818528793566, 79.3481918468942), vec2(186.11818528793566, 116.10507483479371), vec2(161.03015404222646, 147.02753195159806), vec2(117.85540259612226, 147.02753195159806) };
			//drawBezierCurve(letters[i], points, image);
		}
		else if (letters[i].id == 'E') {
			points = {
				vec2(59.00053593969012, 53.9639048228873), vec2(109.60668668470886, 53.9639048228873), vec2(160.2128374297276, 53.9639048228873), vec2(210.8189881747464, 53.9639048228873),
				vec2(210.8189881747464, 53.9639048228873), vec2(204.34331959599993, 89.220322640507), vec2(171.96497670226753, 112.2449220316056), vec2(136.70855888464783, 112.2449220316056),
				vec2(136.70855888464783, 112.2449220316056), vec2(98.57406614314081, 112.2449220316056), vec2(66.91524198038026, 92.09839756439433), vec2(59.00053593969012, 53.9639048228873)
			};
			//drawBezierCurve(letters[i], points, image);
		}
		else if (letters[i].id == 'G') {
			points = {
				vec2(-106.76999344104145, 158.6963836937884), vec2(-149.94474488714565, 158.6963836937884), vec2(-175.03277613285485, 127.77392657698404), vec2(-175.03277613285485, 91.01704358908452),
				vec2(-175.03277613285485, 91.01704358908452), vec2(-175.03277613285485, 53.09327542696597), vec2(-149.94474488714565, 23.92114607149016), vec2(-106.76999344104145, 23.92114607149016),
				vec2(-106.76999344104145, 23.92114607149016), vec2(-63.59524199493726, 23.92114607149016), vec2(-38.507210749228065, 53.09327542696597), vec2(-38.507210749228065, 91.01704358908452),
				vec2(-38.507210749228065, 91.01704358908452), vec2(-38.507210749228065, 127.77392657698404), vec2(-63.59524199493726, 158.6963836937884), vec2(-106.76999344104145, 158.6963836937884)
			};
			//drawBezierCurve(letters[i], points, image);
		}
		else if (letters[i].id == 'A') {
			points = {
				vec2(4.1963044208377624, 80.77886010112692), vec2(-47.55811676949464, 80.77886010112692), vec2(-77.6316317854986, 43.71150438372669), vec2(-77.6316317854986, -0.3496920350698135),
				vec2(-77.6316317854986, -0.3496920350698135), vec2(-77.6316317854986, -45.809656594145565), vec2(-47.55811676949464, -80.77886010112692), vec2(4.1963044208377624, -80.77886010112692),
				vec2(4.1963044208377624, -80.77886010112692), vec2(55.95072561117016, -80.77886010112692), vec2(86.02424062717412, -45.809656594145565), vec2(86.02424062717412, -0.3496920350698135),
				vec2(86.02424062717412, -0.3496920350698135), vec2(86.02424062717412, 43.71150438372669), vec2(55.95072561117016, 80.77886010112692), vec2(4.1963044208377624, 80.77886010112692)
			};
		}
		else if (letters[i].id == 'O') {
			points = {
				vec2(0.0, 79.46100888964659), vec2(-50.91008361760907, 79.46100888964659), vec2(-80.49297004405759, 42.998381433791444), vec2(-80.49297004405759, -0.34398705147033154),
				vec2(-80.49297004405759, -0.34398705147033154), vec2(-80.49297004405759, -45.06230374261343), vec2(-50.91008361760907, -79.46100888964659), vec2(0.0, -79.46100888964659),
				vec2(0.0, -79.46100888964659), vec2(50.91008361760907, -79.46100888964659), vec2(80.49297004405759, -45.06230374261343), vec2(80.49297004405759, -0.34398705147033154),
				vec2(80.49297004405759, -0.34398705147033154), vec2(80.49297004405759, 42.998381433791444), vec2(50.91008361760907, 79.46100888964659), vec2(0.0, 79.46100888964659)
			};
		}
		else if (letters[i].id == 'D') {
			points = {
				vec2(-115.21832444155744, -51.145695239910864), vec2(-156.80932936192454, -51.145695239910864), vec2(-180.9770754643, -80.93384741260621), vec2(-180.9770754643, -116.34240565562142),
				vec2(-180.9770754643, -116.34240565562142), vec2(-180.9770754643, -152.8750451127006), vec2(-156.80932936192454, -180.9770754643), vec2(-115.21832444155744, -180.9770754643),
				vec2(-115.21832444155744, -180.9770754643), vec2(-73.62731952119037, -180.9770754643), vec2(-49.459573418814905, -152.8750451127006), vec2(-49.459573418814905, -116.34240565562142),
				vec2(-49.459573418814905, -116.34240565562142), vec2(-49.459573418814905, -80.93384741260621), vec2(-73.62731952119037, -51.145695239910864), vec2(-115.21832444155744, -51.145695239910864)
			};
		}
		for (int j = 0; j < points.size(); j++) {
			if (j % 4 != 3) {
				bool outline = (i % 4 == 0);
				vec2 pos = vec2(points[j].x(), -points[j].y());
				sPtr<ControlPoint> p = mkS<ControlPoint>(pos, outline);
				letters[i].innerPoints.push_back(std::move(p));
			}
		}
		for (int j = 0; j < letters[i].innerPoints.size(); j++) {
			int prevIdx = (j == 0) ? letters[i].innerPoints.size() - 1 : j - 1;
			int nextIdx = (j == letters[i].innerPoints.size() - 1) ? 0 : j + 1;
			ControlPoint* p = letters[i].innerPoints[j].get();
			p->next = this->letters[i].innerPoints[nextIdx].get();
			p->prev = this->letters[i].innerPoints[prevIdx].get();
		}
	}
	for (int i = 0; i < this->letters.size(); i++) {
		mat3 rot;
		Transform transform = this->letters[i].transform;
		float rad = transform.ori * M_PI / 180.0;
		rot << cos(rad), -sin(rad), 0,
			   sin(rad), cos(rad), 0,
			   0, 0, 1;
		for (int j = 0; j < this->letters[i].controlPoints.size(); j++) {
			vec2 normal = this->letters[i].controlPoints[j]->normal;
			vec3 normalHom = vec3(normal.x(), normal.y(), 1.f);
			normalHom = rot * normalHom;
			float nx = normalHom.x();
			float ny = normalHom.y();
			normal = vec2(nx, ny);
			normal.normalize();
			this->letters[i].controlPoints[j]->normal = normal;
			ControlPoint* p = letters[i].controlPoints[j].get();
			//shrink letter boundary
			float movex = 2.5f / this->letters[i].transform.scale.x();
			float movey = 2.5f / this->letters[i].transform.scale.y();
			if (!p->isFixed) p->pos = p->pos - vec2(p->normal.x()*movex, p->normal.y()*movey);
		}
	}
}
