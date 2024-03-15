#pragma once
#include <opencv2/opencv.hpp>

namespace utilityCore 
{
	extern void thinning(const cv::Mat& src, cv::Mat& dst);
	extern std::vector<cv::Point> extractContour(const cv::Mat& thresh, cv::Mat& outImg);
}
