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

    void assignGrids();
    void calculateClusters();


private:
    Mat keyimg_;
//    Mat disimg_;
    Mat labels_;
//    Mat GridNeighbors_;

    int motion_unit = 1;
    int motion_max = 9;

//    Mat motion_tensor_[150];
//    std::vector<cv::Mat> motion_tensor_;
    // For the grid that has no motion, too few results we make it false.
    std::vector<bool> motion_filter_;

    int mGridNum_;
    cv::Size mGridSize_;

    int w, h, n2;

    float m_, S_;

    int nx, ny;
    float dx, dy;

    vector<Point> centers; // grid centers
//    vector<cv::Vec2f> dist_centers;
    vector<cv::Vec2f> dist_centers;


    void slic_method(); //deprecated
    float dist3d(Point& p1, Vec3f& p1_dis, Point& p2, Vec3f& p2_dis); // 4-D distance between key points 3D Euclidean space
    float dist(Point& p1, Vec2f& p1_dis, Point& p2, Vec2f& p2_dis);

    // Normalize Key Points to Range(0 - 1)
    void NormalizePoints(const vector<cv::Point2d> &kp, const cv::Size &size, vector<cv::Point2f> &npts) {
        const size_t numP = kp.size();
        const int width   = size.width;
        const int height  = size.height;
        npts.resize(numP);

        for (size_t i = 0; i < numP; i++)
        {
            npts[i].x = static_cast<float>(kp[i].x / width);
            npts[i].y = static_cast<float>(kp[i].y / height);
        }
    }

    // Get Neighbor 9
    vector<int> GetNB9(const int idx, const cv::Size& GridSize) {
        vector<int> NB9(9, -1);

        int idx_x = idx % GridSize.width;
        int idx_y = idx / GridSize.width;

        for (int yi = -1; yi <= 1; yi++)
        {
            for (int xi = -1; xi <= 1; xi++)
            {
                int idx_xx = idx_x + xi;
                int idx_yy = idx_y + yi;

                if (idx_xx < 0 || idx_xx >= GridSize.width || idx_yy < 0 || idx_yy >= GridSize.height)
                    continue;

                NB9[xi + 4 + yi * 3] = idx_xx + idx_yy * GridSize.width;
            }
        }
        return NB9;
    }

    void InitializeNeighbors(cv::Mat &neighbor, const cv::Size& GridSize) {
        for (int i = 0; i < neighbor.rows; i++)
        {
            vector<int> NB9 = GetNB9(i, GridSize);
            int *data = neighbor.ptr<int>(i);
            memcpy(data, &NB9[0], sizeof(int) * 9);
        }
    }

};



#endif //PROJECT_GMC_FILTER_H
