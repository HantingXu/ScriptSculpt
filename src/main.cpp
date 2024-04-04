#include "trace_skeleton.cpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "utilities.h"
#include "ConstLetter.h"
#include "LetterAlignment.h"
#include "sceneStruct.h"
#include "GASolver.h"
#include "deformation.h"
using namespace cv;
using Eigen::MatrixXd;



/**
 * This is an example on how to call the thinning funciton above
 */
int main()
{
#define thinning 1
#if thinning
    cv::Mat src = cv::imread("../img/bunny.jpg");
    cv::Mat gray;
    cv::cvtColor(src, gray, COLOR_BGR2GRAY);
    cv::threshold(gray, gray, 170, 255, THRESH_BINARY_INV);
    ImgShape imgShape;
    imgShape.grayScale = gray;

    //extract contour
    std::vector<Point> contour;
    cv::Mat contourImg = cv::Mat::zeros(gray.size(), gray.type());
    utilityCore::extractContour(gray, contourImg, contour);
    cv::Mat ctrImg = contourImg.clone();
    imgShape.contour = contour;
    imgShape.area = cv::contourArea(contour);
    //cv::imshow("contour", contourImg);

    //extract protrusions
    cv::Mat mask;
    std::vector<Protrusion> protrusions;
    utilityCore::genMask(gray, mask);
    cv::Mat protruImg = cv::Mat::zeros(gray.size(), gray.type());
    contourImg.copyTo(protruImg, mask);
    cv::Mat nProtrusion = cv::Mat::zeros(gray.size(), gray.type());;
    utilityCore::genProtrusions(protruImg, nProtrusion, protrusions);
    /*
    for (int j = 0; j < protrusions.size(); j++)
    {
        //cv::line(protruImg, protrusions[j].start, protrusions[j].end, 155, 2);
        std::cout << protrusions[j].axis << std::endl;
    }
    cv::imshow("protrusions", protruImg);*/

    //generate skeleton
    std::vector<Point> centerline;

    nProtrusion = 255 - nProtrusion - contourImg;
    gray.copyTo(nProtrusion, nProtrusion);
    //cv::imshow("no protrusions", nProtrusion);
    cv::Mat skeletonImg = cv::Mat::zeros(gray.size(), gray.type());
    utilityCore::genSkeleton(nProtrusion.clone(), centerline);
    for (int j = 0; j < centerline.size(); j++)
    {
        cv::circle(skeletonImg, centerline[j], 3, 255, 1);
        std::cout << centerline[j] << std::endl;
    }
    cv::imshow("skeleton", skeletonImg);
    imgShape.centerline = centerline;

    //compute protrusion poisition on centerline
    utilityCore::processProtrusions(centerline, protrusions);
    imgShape.protrusions = protrusions;
    /*
    for (int j = 0; j < protrusions.size(); j++)
    {
        //cv::line(protruImg, protrusions[j].start, protrusions[j].end, 155, 2);
        //std::cout << protrusions[j].projection << std::endl;
        //if (protrusions[j].orientation == 0)
        //cv::circle(protruImg, centerline[protrusions[j].projection], 4, 200, -1);
        std::cout << protrusions[j].type << std::endl;
    }
    //cv::imshow("protrusions", protruImg);

    int numLetter = 5;
    std::vector<int> midPoints;
    std::vector<Eigen::Vector2f> normals;
    utilityCore::subdivide(numLetter, centerline, midPoints, normals);

    for (int i = 0; i < numLetter; i++)
    {
        cv::circle(protruImg, centerline[midPoints[i]], 4, 200, -1);
        cv::line(protruImg, centerline[midPoints[i]], Point(normals[i][0] * 14.0f, normals[i][1] * 14.0f) + centerline[midPoints[i]], 155, 1);
    }
    cv::imshow("protrusions", protruImg);*/
    //cv::waitKey();
#endif
#define bezier 0
#if bezier
    // Create a black image
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));

    ConstLetters letters = ConstLetters();
    // Draw the Bezier curve
    /**
    Letter I = letters.getLetter('I');
    Letter U = letters.getLetter('U');
    Letter N = letters.getLetter('N');
    I.setRotate(-45.f);
    U.setRotate(-45.f);
    N.setRotate(45.f);
    I.setTranslate(-200, 200);
    U.setTranslate(0, 0);
    N.setTranslate(200, -200);
    I.setScale(0.5, 0.5);
    U.setScale(0.5, 0.5);
    N.setScale(0.5, 0.5);
    I.drawBezierCurve(image);
    U.drawBezierCurve(image);
    N.drawBezierCurve(image);
    **/
    Letter l1 = letters.getLetter('A');
    Letter l2 = letters.getLetter('B');
    l1.drawBezierCurve(image);
    l2.drawBezierCurve(image);
    //l.setScale(1, 1);
    //l.getContour();
    // Display the image
    cv::imshow("Bezier Curve", image);
    cv::waitKey(0);
#endif
#define initial 1
#if initial
    ConstLetters l = ConstLetters();
    std::vector<Letter> letters;

    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    //Letter l1 = l.getLetter('B');
    Letter l2 = l.getLetter('U');
    //Letter l3 = l.getLetter('N');
    //Letter l4 = l.getLetter('N');
    //Letter l5 = l.getLetter('Y');
    //Letter l6 = l.getLetter('Y');
    //Letter l7 = l.getLetter('O');
    //Letter l8 = l.getLetter('O');
    //letters.push_back(l1);
    letters.push_back(l2);
    //letters.push_back(l3);
    //letters.push_back(l4);
    //letters.push_back(l5);

    //letters.push_back(l6);
    //letters.push_back(l7);
    //letters.push_back(l8);
    //ImgShape img;
    LetterAlignment align = LetterAlignment(letters, imgShape);
    align.initialAlignment();
    cv::Mat mss = contourImg.clone();
    for (int i = 0; i < align.letters.size(); i++) {
        align.letters[i].drawBezierCurve(mss);
    }
    cv::imshow("Bezier Curve1", mss);

    //std::cout << align.smoothFlowScore() << std::endl;

    utilityCore::solveGA(align);
    /*
    cv::Mat canvas = cv::Mat::zeros(imgShape.grayScale.size(), imgShape.grayScale.type());
    cv::Mat canvasTmp = cv::Mat::zeros(imgShape.grayScale.size(), imgShape.grayScale.type());
    for (int i = 0; i < align.letters.size(); i++)
    {
        align.letters[i].getContour(canvas, false);
    }
    cv::bitwise_and(canvas, imgShape.grayScale, canvasTmp);
    cv::bitwise_xor(canvas, canvasTmp, canvasTmp);
    cv::imshow("Bezier Curve", canvasTmp);*/
    
    LetterDeform letterDeform = LetterDeform(align.letters, imgShape, ctrImg);
    letterDeform.updateNormal();
    Deform deform = Deform(40000, 10, 60, 0.025, &letterDeform);
    std::vector<std::vector<bool>> sol;
    
    for (int i = 0; i < sol.size(); i++)
    {
        for (int j = 0; j < sol[i].size(); j++)
        {
            std::cout << sol[i][j] << " ";
        }
        std::cout << std::endl;
    }
    for (int i = 0; i < 6; i++)
    {
        deform.localStep(sol);
        letterDeform.updateLetter(sol, 10);
        
    }
    letterDeform.splitLetter();
    deform.setStep(5);
    sol.clear();
    for (int i = 0; i < 12; i++)
    {
        deform.localStep(sol);
        letterDeform.updateLetter(sol, 5);
    }
    cv::Mat canvas = cv::Mat::zeros(contourImg.size(), cv::COLOR_BGR2GRAY);
    for (int i = 0; i < letterDeform.letters.size(); i++) {
        letterDeform.letters[i].drawBezierCurve(contourImg);
        letterDeform.letters[i].getContour(canvas, false);
    }
    cv::imshow("Bezier Curve", canvas);
    cv::imshow("Bezier Curvecc", contourImg);
    /*
    std::vector<bool> v = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    sol.push_back(v);
    std::vector<std::vector<vec2>> ptsPos;
    for (int j = 0; j < letterDeform.letters.size(); j++)
    {
        ptsPos.push_back(std::vector<vec2>());
        letterDeform.letters[j].update(v, ptsPos[j], 100);
    }
    float s = letterDeform.fitScore(ptsPos);
    std::cout << s << std::endl;
    letterDeform.updateLetter(sol, 100);
    for (int i = 0; i < letterDeform.letters.size(); i++) {
        letterDeform.letters[i].drawBezierCurve(contourImg);
    }
    cv::imshow("Bezier Curve", contourImg);
   */
    cv::waitKey(0);
#endif
    return 0;
}
