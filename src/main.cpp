
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "utilities.h"
#include "ConstLetter.h"
using namespace cv;
using Eigen::MatrixXd;



/**
 * This is an example on how to call the thinning funciton above
 */
int main()
{
#define thinning 0
#if thinning
    cv::Mat src = cv::imread("../img/cat.png");
    if (!src.data)
        return -1;

    cv::Mat bw;
    cv::Mat ct = src.clone();
    cv::cvtColor(src, bw, COLOR_BGR2GRAY);
    cv::threshold(bw, bw, 170, 255, THRESH_BINARY_INV);
    
    utilityCore::extractContour(bw, ct);
    utilityCore::thinning(bw, bw);

    cv::imshow("src", src);
    cv::imshow("dst", bw);
    cv::imshow("contour", ct);
    cv::waitKey();
 #endif
#define bezier 1
#if bezier
    // Create a black image
    cv::Mat image(1200, 1200, CV_8UC3, cv::Scalar(0, 0, 0));

    ConstLetters letters = ConstLetters();
    // Draw the B¨¦zier curve
    letters.getLetter('A').drawBezierCurve(image);

    // Display the image
    cv::imshow("Bezier Curve", image);
    cv::waitKey(0);
#endif
    return 0;
}