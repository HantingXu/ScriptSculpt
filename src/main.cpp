
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "utilities.h"
using namespace cv;
using Eigen::MatrixXd;



/**
 * This is an example on how to call the thinning funciton above
 */
int main()
{
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
    return 0;
}