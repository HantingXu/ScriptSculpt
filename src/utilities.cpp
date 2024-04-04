#include "utilities.h"
#include "trace_skeleton.cpp"
#include <iostream>


using namespace cv;
#define PI 3.141592653589793238462643
#define TORADIAN 0.01745329252
#define TODEGREE 57.295779513

void utilityCore::extractContour(const cv::Mat& thresh, cv::Mat& outImg, std::vector<cv::Point>& contour)
{
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    outImg.setTo(Scalar::all(0));
    // draw contours on the output image
    drawContours(outImg, contours, 0, Scalar(255, 255, 255), 1);
    contour = contours[0];
}


void utilityCore::genMask(const cv::Mat& input, cv::Mat& mask)
{
    // Detect corners using Harris corner detector
    mask = input.clone();

    cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(5, 5));
    // Dilate the result for marking the corners
    erode(mask, mask, kernel, cv::Point(-1, -1), 13, 1, 1);
    dilate(mask, mask, kernel, cv::Point(-1, -1), 13, 1, 1);
    mask.convertTo(mask, CV_8U);
    cv::bitwise_not(mask, mask);
}



bool findNextPoint(std::vector<Point>& neighbors, Mat& img, Point inPoint, int flag, Point& outPoint, int& outFlag)
{
    int i = flag;
    int count = 1;
    bool success = false;

    while (count <= 7)
    {
        Point tmpPoint = inPoint + neighbors[i];
        if (tmpPoint.x > 0 && tmpPoint.y > 0 && tmpPoint.x < img.cols && tmpPoint.y < img.rows)
        {
            if (img.at<uchar>(tmpPoint) == 255)
            {
                outPoint = tmpPoint;
                outFlag = i;
                success = true;
                img.at<uchar>(tmpPoint) = 0;
                break;
            }
        }
        if (count % 2)
        {
            i += count;
            if (i > 7)
            {
                i -= 8;
            }
        }
        else
        {
            i += -count;
            if (i < 0)
            {
                i += 8;
            }
        }
        count++;
    }
    return success;
}


//find the first for the input image
bool findFirstPoint(Mat& inImg, Point& outPoint)
{
    bool success = false;
    for (int i = 0; i < inImg.rows; i++)
    {
        uchar* data = inImg.ptr(i);
        for (int j = 0; j < inImg.cols; j++)
        {
            if (data[j] == 255)
            {
                success = true;
                outPoint.x = j;
                outPoint.y = i;
                data[j] = 0;
                break;
            }
        }
        if (success)
            break;
    }
    return success;
}
//find the curves
void findLines(Mat& inImg, std::vector<std::deque<Point>>& outLines)
{
    std::vector<Point> neighbors = { Point(-1,-1),Point(0,-1),Point(1,-1),Point(1,0),Point(1,1),Point(0,1),Point(-1,1),Point(-1,0) };
    Point start;
    while (findFirstPoint(inImg, start))
    {
        std::deque<Point> line;
        line.push_back(start);
        //the first point might not be the start point
        Point tmpPoint = start;
        int flag1 = 0;
        Point nextPoint;
        int flag2;
        while (findNextPoint(neighbors, inImg, tmpPoint, flag1, nextPoint, flag2))
        {
            line.push_back(nextPoint);
            tmpPoint = nextPoint;
            flag1 = flag2;
        }
        //search the other side
        tmpPoint = start;
        flag1 = 0;
        //cout << "flag:" << this_flag << endl;
        while (findNextPoint(neighbors, inImg, tmpPoint, flag1, nextPoint, flag2))
        {
            line.push_front(nextPoint);
            tmpPoint = nextPoint;
            flag1 = flag2;
        }
        if (line.size() > 10)
        {
            outLines.push_back(line);
        }
    }
}

Scalar random_color(RNG& rng)
{
    int icolor = (unsigned)rng;
    return Scalar(icolor & 0xFF, (icolor >> 8) & 0xFF, (icolor >> 16) & 0xFF);
}


void utilityCore::genProtrusions(cv::Mat& protruImg, cv::Mat& nProImg, std::vector<Protrusion>& protrusions)
{
    std::vector<std::deque<Point>> contours;
    std::vector<std::vector<Point>> proContours;
    std::vector<std::vector<Point>> cProContours;
    std::vector<Vec4i> hierarchy;
    std::vector<Point> endPoints;
    findLines(protruImg.clone(), contours);
    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<Point> v(contours[i].begin(), contours[i].end());
        proContours.push_back(v);
        /*
        protrusions.push_back({
            contours[i][0],
            contours[i][contours[i].size() - 1],
            -1, -1});*/
    }
    cv::polylines(protruImg, proContours, 1, 255, 1);
    cv::Mat sav = protruImg.clone();
    cv::floodFill(protruImg, Point(0, 0), 255);
    protruImg = 255 - protruImg + sav;

    findContours(protruImg, cProContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    int contourNum = 0;
    for (int i = 0; i < cProContours.size(); i++)
    {
        double area = cv::contourArea(cProContours[i]);
        std::cout << area << std::endl;
        cv::drawContours(nProImg, cProContours, i, 255, -1);
        //needs further configuration
        if (area < 1000 || area > 7000)
        {
            cv::drawContours(protruImg, cProContours, i, 0, -1);
            ++contourNum;
        }
    }
    Mat t = cv::Mat::zeros(protruImg.size(), protruImg.type());
    for (int i = 0; i < proContours.size(); i++)
    {
        Point start = proContours[i][0];
        Point end = proContours[i][proContours[i].size() - 1];
        if (protruImg.at<uchar>(start) > 0 && protruImg.at<uchar>(end) > 0)
        {
            cv::RotatedRect rect = minAreaRect(proContours[i]);
            cv::Point2f p[4];
            rect.points(p);
            Vec2f v1 = Vec2f((p[1] - p[0]).x, (p[1] - p[0]).y);
            Vec2f v2 = Vec2f((p[3] - p[0]).x, (p[3] - p[0]).y);
            if (cv::norm(v1) < cv::norm(v2))
            {
                v1 = v2;
            }
            float rad = rect.angle * TORADIAN;
            protrusions.push_back({
            start,
            end,
            cv::normalize(v1),
            Point(rect.center.x, rect.center.y),
            -1, -1 });

            cv::line(t, p[0], p[1], 155, 2);
            cv::line(t, p[1], p[2], 155, 2);
            cv::line(t, p[2], p[3], 155, 2);
            cv::line(t, p[3], p[0], 155, 2);
        }

    }

    cv::imshow("protrus", t);
}

void utilityCore::getLongestPath(const std::vector<std::vector<Vec3i>>& graph, int& maxPath, std::vector<int>& path)
{
    int start = 0;
    int level = 0;
    std::queue<int> waitlist;
    std::vector<int> visited(graph.size(), 0);
    std::vector<int> distance(graph.size(), 0);
    std::vector<int> finalList(graph.size(), -1);

    waitlist.push(0);
    int maxDistance = -1;
    int maxIdx = -1;
    while (!waitlist.empty())
    {
        int currNode = waitlist.front();
        //std::cout << currNode << std::endl;
        visited[currNode] = 1;
        int d = distance[currNode];
        for (int i = 0; i < graph[currNode].size(); i++)
        {
            int idx = graph[currNode][i][0];
            // the node is not visited before
            if (visited[idx] == 0)
            {
                //visit its children later
                waitlist.push(idx);
                distance[idx] = d + graph[currNode][i][1];
                if (distance[idx] > maxDistance)
                {
                    maxDistance = distance[idx];
                    maxIdx = idx;
                }
            }
        }
        waitlist.pop();
    }
    std::cout << maxIdx << std::endl;

    waitlist.push(maxIdx);
    std::fill(distance.begin(), distance.end(), 0);
    std::fill(visited.begin(), visited.end(), 0);
    maxDistance = -1;
    maxIdx = -1;
    while (!waitlist.empty())
    {
        int currNode = waitlist.front();
        visited[currNode] = 1;
        int d = distance[currNode];
        for (int i = 0; i < graph[currNode].size(); i++)
        {
            int idx = graph[currNode][i][0];
            // the node is not visited before
            if (visited[idx] == 0)
            {
                //visit its children later
                waitlist.push(idx);
                distance[idx] = d + graph[currNode][i][1];
                finalList[idx] = currNode;
                if (distance[idx] > maxDistance)
                {
                    maxDistance = distance[idx];
                    maxIdx = idx;
                }
            }
        }
        waitlist.pop();
    }
    int tmpIdx = maxIdx;
    while (tmpIdx != -1)
    {
        path.push_back(tmpIdx);
        tmpIdx = finalList[tmpIdx];
    }
    maxPath = maxDistance;
    //std::cout << maxDistance << std::endl;
}

void utilityCore::genGraph(const std::vector<cv::Point>& endpoints, const std::vector<int>& distances, std::vector<std::vector<cv::Vec3i>>& graph)
{
    std::vector<Point> points;
    for (int idx = 0; idx < distances.size(); idx++)
    {
        int pointIdx = idx * 2;
        Vec2i edgePoints;
        for (int p = 0; p < 2; p++)
        {
            int graphIdx = -1;
            for (int i = 0; i < points.size(); i++)
            {
                int dis = cv::norm(endpoints[pointIdx + p] - points[i]);
                if (dis < 5)
                {
                    graphIdx = i;
                    break;
                }
            }
            if (graphIdx == -1)
            {
                points.push_back(endpoints[pointIdx + p]);
                graphIdx = points.size() - 1;
            }
            edgePoints[p] = graphIdx;
        }
        for (int i = graph.size(); i < points.size(); i++)
        {
            graph.push_back(std::vector<Vec3i>());
        }
        for (int p = 0; p < 2; p++)
        {
            int id = edgePoints[1 - p];
            if (id != edgePoints[p])
                graph[edgePoints[p]].push_back(Vec3i(id, distances[idx], idx));
        }

    }
}

void utilityCore::genSkeleton(const cv::Mat& img, std::vector<cv::Point>& centerline)
{
    img /= 255;
    int w = img.size().width;
    int h = img.size().height;
    skeleton_tracer_t* T = new skeleton_tracer_t();
    T->W = w; // width of image
    T->H = h; // height of image

    // allocate the input image
    T->im = (unsigned char*)malloc(sizeof(unsigned char) * w * h);
    std::copy(img.data, img.data + w * h, T->im);

    T->thinning_zs(); // perform raster thinning

    cv::Mat image_as_mat(Size(w, h), CV_8U, T->im);
    image_as_mat *= 255;

    // run the algorithm
    skeleton_tracer_t::polyline_t* p = (skeleton_tracer_t::polyline_t*)T->trace_skeleton(0, 0, w, h, 0);
    std::vector<Point> endpoints;
    std::vector<std::vector<Point>> ptPath;
    std::vector<int> lengths;
    cv::Mat dstImage = cv::Mat::zeros(img.size(), img.type());
    // print out points in every polyline
    skeleton_tracer_t::polyline_t* it = p; //iterator
    int i = 0;
    while (it) {
        skeleton_tracer_t::point_t* jt = it->head;
        Point start = Point(0, 0);
        int j = 0;
        int len = 0;
        ptPath.push_back(std::vector<Point>());
        while (jt) {
            ptPath[i].push_back(Point(jt->x, jt->y));
            if (j != 0)
            {
                len += cv::norm(Point(jt->x, jt->y) - start);
                //cv::line(dstImage, Point(jt->x, jt->y), start, 255, 1);
            }
            start.x = jt->x;
            start.y = jt->y;
            jt = jt->next;
            if (jt == nullptr)
            {
                endpoints.push_back(start);
                lengths.push_back(len);
            }
            if (j == 0)
            {
                endpoints.push_back(start);
            }
            ++j;
        }
        it = it->next;
        ++i;
    }
    std::vector<std::vector<cv::Vec3i>> graph;
    utilityCore::genGraph(endpoints, lengths, graph);
    /*
    for (int i = 0; i < graph.size(); i++)
    {
        std::cout << i << ": ";
        for (Vec3i j : graph[i])
        {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }*/
    int maxPath = 0;
    std::vector<int> path;
    utilityCore::getLongestPath(graph, maxPath, path);
    int st = path[0];
    //bool bg = 0;
    for (int i = 1; i < path.size(); i++)
    {
        int ed = path[i];
        int id = -1;
        for (int j = 0; j < graph[st].size(); j++)
        {
            if (graph[st][j][0] == ed)
            {
                id = graph[st][j][2];
                break;
            }
        }
        for (int j = 0; j < ptPath[id].size() - 1; j++)
        {
            cv::line(dstImage, ptPath[id][j], ptPath[id][j + 1], 255);
        }
        st = ed;
    }
    std::vector<std::deque<Point>> skeleton;
    findLines(dstImage, skeleton);

    centerline = std::vector<Point>(skeleton[0].begin(), skeleton[0].end());
    if (skeleton[0][0].x > skeleton[0][skeleton[0].size() - 1].x)
        std::reverse(centerline.begin(), centerline.end());
    free(T->im);
    T->destroy_polylines(p);
    T->destroy_rects();
    delete T;

}

int findClosestPoint(const std::vector<cv::Point>& line, const Point& point)
{
    float minDis2 = FLT_MAX;
    int res = -1;
    for (int i = 0; i < line.size(); i++)
    {
        float distance2 = (line[i].x - point.x) * (line[i].x - point.x) + (line[i].y - point.y) * (line[i].y - point.y);
        if (distance2 < minDis2)
        {
            minDis2 = distance2;
            res = i;
        }
    }
    return res;
}

void utilityCore::processProtrusions(const std::vector<cv::Point>& centerline, std::vector<Protrusion>& protrustions)
{
    cv::RotatedRect rect = minAreaRect(centerline);
    cv::Point2f p[4];
    rect.points(p);
    Vec2f v1 = Vec2f((p[1] - p[0]).x, (p[1] - p[0]).y);
    Vec2f v2 = Vec2f((p[3] - p[0]).x, (p[3] - p[0]).y);
    if (cv::norm(v1) < cv::norm(v2))
    {
        v1 = v2;
    }
    float rad = rect.angle * TORADIAN;
    Vec2f mainAxis = cv::normalize(v1);
    //std::cout << mainAxis << std::endl;
    for (int i = 0; i < protrustions.size(); i++)
    {
        int startIdx = findClosestPoint(centerline, protrustions[i].start);
        int endIdx = findClosestPoint(centerline, protrustions[i].end);
        int midIdx = (startIdx + endIdx) / 2;
        protrustions[i].projection = midIdx;
        protrustions[i].orientation = STRAIGHT;
        protrustions[i].type = ASCEND;
        int xSide = (protrustions[i].center.x - centerline[midIdx].x) * protrustions[i].axis[0];
        int ySide = (protrustions[i].center.y - centerline[midIdx].y) * protrustions[i].axis[1];
        if (ySide < 0.0f)
        {
            //flip
            protrustions[i].axis = -protrustions[i].axis;
        }
        else if (ySide == 0.0f)
        {
            if (xSide < 0.0f)
                protrustions[i].axis = -protrustions[i].axis;
        }
        if (protrustions[i].axis[1] > 0)
        {
            protrustions[i].orientation = STRAIGHT;
            protrustions[i].type = DESCEN;
        }
        //std::cout << protrustions[i].axis << std::endl;
        // costheta
        float precursor = protrustions[i].axis.dot(mainAxis);
        if (precursor < -0.70710678118)
        {
            protrustions[i].orientation = LEFT;
        }
        else if (precursor > 0.70710678118)
        {
            protrustions[i].orientation = RIGHT;
        }
    }
}

int computeOverlap(const int area1, const int area2, const Mat& img)//const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2)
{
    int tot = cv::countNonZero(img);
    return area1 + area2 - tot;
}

//centerline size should be bigger than 0
void utilityCore::subdivide(const int number, const std::vector<cv::Point>& centerline, std::vector<int>& midPoints, std::vector<Eigen::Vector2f>& normals)
{
    int last = centerline.size();
    int interval = last / number;
    last = last - 1;
    int first = interval / 2;
    for (int i = 0; i < number; i++)
    {
        int index = first + i * interval;
        midPoints.push_back(index);
        int leftEnd1 = max(0, index - 3);
        int rightEnd1 = min(index + 3, last);
        int leftEnd2 = max(0, index - 5);
        int rightEnd2 = min(index + 5, last);
        Eigen::Vector2f tangent1 = Eigen::Vector2f((centerline[rightEnd1] - centerline[leftEnd1]).x, (centerline[rightEnd1] - centerline[leftEnd1]).y).normalized();
        Eigen::Vector2f tangent2 = Eigen::Vector2f((centerline[rightEnd2] - centerline[leftEnd2]).x, (centerline[rightEnd2] - centerline[leftEnd2]).y).normalized();
        Eigen::Vector2f tangent = (tangent1 + tangent2) / 2.0f;
        normals.push_back(Eigen::Vector2f(tangent[1], -tangent[0]));
    }
}

void utilityCore::solveGA(LetterAlignment& align)
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GASolver solver = GASolver(&align);

    GA_Type ga_obj;
    ga_obj.problem_mode = EA::GA_MODE::SOGA;
    ga_obj.multi_threading = false;
    ga_obj.idle_delay_us = 1; // switch between threads quickly
    ga_obj.verbose = false;
    ga_obj.population = 30;
    ga_obj.generation_max = 2000;
    ga_obj.calculate_SO_total_fitness = std::bind(&GASolver::calculateSOTotalFitness, &solver, _1);
    ga_obj.init_genes = std::bind(&GASolver::initGenes, &solver, _1, _2);
    ga_obj.eval_solution = std::bind(&GASolver::evalSolution, &solver, _1, _2);
    ga_obj.mutate = std::bind(&GASolver::mutate, &solver, _1, _2, _3);
    ga_obj.crossover = std::bind(&GASolver::crossover, &solver, _1, _2, _3);
    ga_obj.SO_report_generation = std::bind(&GASolver::SOReportGeneration, &solver, _1, _2, _3);
    ga_obj.best_stall_max = 10;
    ga_obj.elite_count = 20;
    ga_obj.crossover_fraction = 0.7;
    ga_obj.mutation_rate = 0.3;
    ga_obj.solve();

    int idx = ga_obj.last_generation.best_chromosome_index;
    std::cout << ga_obj.last_generation.chromosomes[idx].genes.to_string() << std::endl;
    std::cout << ga_obj.last_generation.chromosomes[idx].middle_costs.objective1 << std::endl;
    align.setLetters(ga_obj.last_generation.chromosomes[idx].genes);
}