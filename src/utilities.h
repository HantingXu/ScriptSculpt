#pragma once
#include <opencv2/opencv.hpp>
#include "sceneStruct.h"
#include "LetterAlignment.h"
#include "GASolver.h"
#include "deformation.h"
#include <Eigen/Dense>

namespace utilityCore
{
	extern void extractContour(const cv::Mat& thresh, cv::Mat& outImg, std::vector<cv::Point>& contour);
	extern void extractContour(const cv::Mat& thresh, cv::Mat& outImg, std::vector<cv::Point>& contour, const cv::Scalar& fontColor, const cv::Scalar& backColor);

	extern void genMask(const cv::Mat& input, cv::Mat& mask);
	extern void genProtrusions(cv::Mat& preData, cv::Mat& nProImg, std::vector<Protrusion>& protrustions);

	//vec2.x: point index
	//vec2.y: egde length
	extern void getLongestPath(const std::vector<std::vector<cv::Vec3i>>& graph, int& maxPath, std::vector<int>& path);
	extern void genGraph(const std::vector<cv::Point>& endpoints, const std::vector<int>& distances, std::vector<std::vector<cv::Vec3i>>& graph);
	extern void genSkeleton(const cv::Mat& img, std::vector<cv::Point>& centerline);
	extern void processProtrusions(const std::vector<cv::Point>& centerline, std::vector<Protrusion>& protrustions);
	extern void subdivide(const int number, const std::vector<cv::Point>& centerline, std::vector<int>& midPoints, std::vector<Eigen::Vector2f>& normals);
	extern void solveGA(LetterAlignment& letterAlign);
	extern void genMayaImage(const std::string& shapePath, const std::string& word, const vec3& fontColor, const vec3& backColor, cv::Mat& outputImg);
}

