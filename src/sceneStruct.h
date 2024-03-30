#pragma once
#include <opencv2/opencv.hpp>
#include "openGA.hpp"
#include <Eigen/Dense>
#define M_PI       3.14159265358979323846
#define TORADIAN 0.01745329252
#define TODEGREE 57.295779513

#define STRAIGHT 0
#define RIGHT 1
#define LEFT 2
#define ASCEND 0
#define DESCEN 1

using vec2 = Eigen::Vector2f;
using vec3 = Eigen::Vector3f;
using mat3 = Eigen::Matrix3f;

struct Protrusion
{
	cv::Point start;
	cv::Point end;
	cv::Vec2f axis;
	cv::Point center;
	int orientation;
	int type;
	//the position on the centerline
	int projection;
};

struct ImgShape
{
	int area;
	cv::Mat grayScale;
	std::vector<cv::Point> centerline;
	std::vector<cv::Point> contour;
	std::vector<Protrusion> protrusions;
};


struct GASolution
{
	std::vector<float> var = std::vector<float>(30);
	GASolution()
	{
		//maximum 10 letters
		var.reserve(30);
	}
	std::string to_string() const
	{
		std::ostringstream out;
		out << "{";
		for (unsigned long i = 0; i < var.size(); i++)
			out << (i ? "," : "") << std::setprecision(10) << var[i];
		out << "}";
		return out.str();
	}
};

struct GACost
{
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};

typedef EA::Genetic<GASolution, GACost> GA_Type;
typedef EA::GenerationType<GASolution, GACost> Generation_Type;

