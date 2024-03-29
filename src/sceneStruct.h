#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

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
	std::vector<cv::Point> centerline;
	std::vector<cv::Point> contour;
	std::vector<Protrusion> protrusions;
};


