//
// Created by hd on 19-1-27.
//
#ifndef PROJECT_GMC_FILTER_H
#define PROJECT_GMC_FILTER_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <eigen3/Eigen/Dense>

#include <math.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <map>

#define DEFAULT_M 15
#define USE_DEFAULT_S -1
using namespace std;
using namespace cv;

class GMC {
public:
    GMC(Mat& keyImg, float m = DEFAULT_M, float S = USE_DEFAULT_S);

    void viewGridResults(Mat& input, Mat& output);

    Mat getLabels();

private:
    Mat keyimg_;
    Mat disimg_;
    Mat labels_;

    int w, h, n2;

    float m_, S_;

    int nx, ny;
    float dx, dy;

    vector<Point> centers; // grid centers
//    vector<cv::Vec2f> dist_centers;
    vector<cv::Vec3f> dist_centers;

    void assignGrids();
    void calculateClusters();
    float dist3d(Point& p1, Vec3f& p1_dis, Point& p2, Vec3f& p2_dis); // 4-D distance between key points 3D Euclidean space
    float dist(Point& p1, Vec2f& p1_dis, Point& p2, Vec2f& p2_dis);
};



#endif //PROJECT_GMC_FILTER_H
