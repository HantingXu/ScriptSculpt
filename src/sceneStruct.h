#pragma once
#include <opencv2/opencv.hpp>
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3
struct Protrusion
{
	cv::Point start;
	cv::Point end;
	cv::Vec2f axis;
	cv::Point center;
	int orientation;
	//the position on the centerline
	int projection;
};