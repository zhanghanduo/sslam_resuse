//
// Created by hd on 19-1-27.
//
#include <algorithm>
#include <math.h>
#include "gmc_filter.h"

GMC::GMC(Mat &keyimg, float m, float S): m_(m) {
    keyimg_ = keyimg.clone();
    w = keyimg_.cols;
    h = keyimg_.rows;
    if(S == USE_DEFAULT_S) {
        nx = 15;
        ny = 5;
        dx = w / float(nx);
        dy = h / float(ny);
        S_ = (dx + dy + 1)/2;   // default window size
    }
    else
        S_ = S;

    motion_max = 9;
    motion_unit = 1;

    mGridNum_ = nx*ny;
    mGridSize_ = cv::Size(nx, ny);

//    GridNeighbors_ = cv::Mat::zeros(mGridNum_, 9, CV_32SC1);
//    InitializeNeighbors(GridNeighbors_, mGridSize_);

//    assignGrids();
//    calculateClusters();

}

void GMC::assignGrids() {

    int pattern_size = static_cast<int>(ceil((motion_max * 2) / motion_unit));
//    MotionPatterns_ = cv::Mat::zeros(pattern_size, pattern_size, CV_8UC1);
//    std::vector<cv::Mat> motion_tensor_;
//    mGridNum_ = 150;
    uint motion_tensors[mGridNum_][pattern_size][pattern_size] = {{{0}}};

    motion_filter_.reserve(mGridNum_);
    for (int j = 0; j < mGridNum_ ; j++) {
        motion_filter_[j] = false;
    }
    Mat labels = -1 * Mat::ones(this->keyimg_.size(), CV_32FC2);

    //TODO: shift left and right of the motion pattern

    for(int r = 5; r < keyimg_.rows - 5; r++) {
        for (int c = 2; c < keyimg_.cols - 2; c++) {
            if((keyimg_.at<cv::Vec2f>(r, c)[0] == -1.0f) && (keyimg_.at<cv::Vec2f>(r, c)[1] == -1.0f))
                continue;
            // 1.1 Find the grid index that pixel belongs to
            int x = static_cast<int>(floor(c * nx / w));
            int y = static_cast<int>(floor(r * ny / h));

            if(x >= nx || y >= ny)
                continue;

            int layer = x + y * nx;
            if(layer >= mGridNum_ )
                continue;

            // 1.2 Which motion pattern to match and summarize the motion statistics
            float dist_x = keyimg_.at<cv::Vec2f>(r, c)[0] / motion_unit;
            float dist_z = keyimg_.at<cv::Vec2f>(r, c)[1] / motion_unit;

//            cout << "x: " << keyimg_.at<cv::Vec2f>(r, c)[0] << endl;
//            cout << "z: " << keyimg_.at<cv::Vec2f>(r, c)[1] << endl;

            int cell_m_x = static_cast<int>(floor(dist_x));
            int cell_m_z = static_cast<int>(floor(dist_z));

//            cout << "cell z: " << cell_m_z << " x: " << cell_m_x << endl;
//            cout << "layer: " << layer << endl;

            int zz = 0, xx = 0;

            if(cell_m_x >= 0) {
                if(cell_m_z >= 0) {
                    if(cell_m_x < motion_max) {
                        if(cell_m_z < motion_max) {
                            xx = pattern_size / 2 + cell_m_x;
                            zz = pattern_size / 2 + cell_m_z;
//                            cout << "z: " << pattern_size / 2 + cell_m_z  << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z , pattern_size / 2 + cell_m_x ) += 1;
                        } else {
                            zz = pattern_size - 1;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << pattern_size << " x: " << pattern_size / 2 + cell_m_x << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size - 1, pattern_size / 2 + cell_m_x ) += 1;
                        }
                    } else {
                        if(cell_m_z < motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = pattern_size - 1;
//                            cout << "z: " << pattern_size / 2 + cell_m_z  << " x: " << pattern_size - 1  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z , pattern_size - 1 ) += 1;
                        } else {
                            zz = pattern_size - 1;
                            xx = pattern_size - 1;
//                            cout << "z: " << pattern_size << " x: " << pattern_size << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size - 1, pattern_size - 1) += 1;
                        }
                    }
                }
                else {
                    if(cell_m_x < motion_max) {
                        if(cell_m_z > - motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << pattern_size / 2 + cell_m_z  << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, pattern_size / 2 + cell_m_x) += 1;
                        } else {
                            zz = 0;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << 0  << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(0, pattern_size / 2 + cell_m_x) += 1;
                        }
                    } else {
                        if(cell_m_z > - motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = pattern_size - 1;
//                            cout << "z: " << pattern_size / 2 + cell_m_z  << " x: " << pattern_size - 1  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, pattern_size - 1) += 1;
                        } else {
                            zz = 0;
                            xx = pattern_size - 1;
//                            cout << "z: " << 0  << " x: " << pattern_size - 1  << endl;
//                            motion_tensor_[layer].at<uint>(0, pattern_size - 1) += 1;
                        }
                    }
                }
            }
            else {
                if(cell_m_z >= 0) {
                    if(cell_m_x > - motion_max) {
                        if(cell_m_z < motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << pattern_size / 2 + cell_m_z << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, pattern_size / 2 + cell_m_x) += 1;
                        } else {
                            zz = pattern_size - 1;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << pattern_size << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size - 1, pattern_size / 2 + cell_m_x) += 1;
                        }
                    } else {
                        if(cell_m_z < motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = 0;
//                            cout << "z: " << pattern_size / 2 + cell_m_z << " x: " << 0  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, 0) += 1;
                        } else {
                            zz = pattern_size - 1;
                            xx = 0;
//                            cout << "z: " << pattern_size - 1 << " x: " << 0  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size - 1, 0) += 1;
                        }
                    }
                } else {
                    if(cell_m_x > - motion_max) {
                        if(cell_m_z > - motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << pattern_size / 2 + cell_m_z << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, pattern_size / 2 + cell_m_x) += 1;
                        } else {
                            zz = 0;
                            xx = pattern_size / 2 + cell_m_x;
//                            cout << "z: " << 0 << " x: " << pattern_size / 2 + cell_m_x  << endl;
//                            motion_tensor_[layer].at<uint>(0, pattern_size / 2 + cell_m_x) += 1;
                        }
                    } else {
                        if(cell_m_z > - motion_max) {
                            zz = pattern_size / 2 + cell_m_z;
                            xx = 0;
//                            cout << "z: " << pattern_size / 2 + cell_m_z << " x: " << 0 << endl;
//                            motion_tensor_[layer].at<uint>(pattern_size / 2 + cell_m_z, 0) += 1 ;
                        }
                    }
                }
            }
            motion_tensors[layer][zz][xx] ++;
        }
    }

    uint sum_;
    // 2. Since we get the motion tensor, we only pick the consensus winner.
    for (int i = 0; i < mGridNum_ ; i++) {
//        cv::Mat mPattern = motion_tensor_[i].clone();
        sum_ = 0;
        uint maxVal = 0;
        Point maxLoc;
        for ( int ii = 0; ii < pattern_size; ii++ )
        {
            for ( int jj = 0; jj < pattern_size; jj++ )
            {
                sum_ += motion_tensors[i][ii][jj];
                if(maxVal < motion_tensors[i][ii][jj])
                {
                    maxVal = motion_tensors[i][ii][jj];
                    maxLoc.x = jj ;
                    maxLoc.y = ii ;
                }
            }
        }
//        cout << "maxval: " << maxVal << " max loc: " << maxLoc.x << endl;
        if((sum_ < 3) || (sum_ > 50000)) {
            motion_filter_[i] = false;
//            cout << "pattern " << i << " has " << sum_ << " features, too few." << endl;
            continue;
        }
//        cout << "pattern " << i << " has " << sum_ << " features." << endl;

//        if (sum(mPattern)[0] < 5) {
//            motion_filter_[i] = false;
////            cout << "pattern " << i << " has " << sum(mPattern)[0] << " features, too few." << endl;
//            continue;
//        }

//        minMaxLoc( mPattern, &minVal, &maxVal, &minLoc, &maxLoc );
        if (maxVal < 2) {
            motion_filter_[i] = false;
//            cout << "pattern " << i << " has max " << maxVal << " statistics, too few." << endl;
            continue;
        } else if ( ((maxLoc.x == pattern_size/2)||(maxLoc.x == pattern_size/2 - 1))
        && ((maxLoc.y == pattern_size/2)||(maxLoc.y == pattern_size/2 - 1) ) ) {
            motion_filter_[i] = false;
//            cout << "pattern " << i << " static " << endl;
            continue;
        }
        else {
            motion_filter_[i] = true;
            cout << "motion " << i << " is true!" << maxVal << endl;

            int grid_idx = i % ny;
            int grid_idy = i / nx;

            for(int r = int(grid_idy * dy); r < static_cast<int>(int(grid_idy * dy) + floor(dy)); r++) {
                for(int c = int(grid_idx * dx); c < static_cast<int>(int(grid_idx * dx) + floor(dx)); c++) {
                    if ((keyimg_.at<cv::Vec2f>(r, c)[0] == -1.0f) || (keyimg_.at<cv::Vec2f>(r, c)[1] == -1.0f))
                        continue;
                    labels.at<Vec2f>(r, c)[0] = keyimg_.at<cv::Vec2f>(r, c)[0];
                    labels.at<Vec2f>(r, c)[1] = keyimg_.at<cv::Vec2f>(r, c)[1];
//                    labels.at<float>(r, c) = keyimg_.at<cv::Vec2f>(r, c)[0];
                }
            }
        }




    }
    // For 0059
//        for(int r = int(2 * dy); r < static_cast<int>(int(2 * dy) + floor(dy))*2; r++) {
//            for(int c = int(7 * dx); c < static_cast<int>(int(7 * dx) + floor(dx)); c++) {
//                if ((keyimg_.at<cv::Vec2f>(r, c)[0] == -1.0f) || (keyimg_.at<cv::Vec2f>(r, c)[1] == -1.0f))
//                    continue;
//                labels.at<Vec2f>(r, c)[0] = keyimg_.at<cv::Vec2f>(r, c)[0];
//                labels.at<Vec2f>(r, c)[1] = keyimg_.at<cv::Vec2f>(r, c)[1];
////                    labels.at<float>(r, c) = keyimg_.at<cv::Vec2f>(r, c)[0];
//            }
//        }
    // 0051
//    for(int r = 150; r < static_cast<int>(150 + floor(dy)); r++) {
//        for(int c = 415; c < static_cast<int>(435 + floor(dx)); c++) {
//            if ((keyimg_.at<cv::Vec2f>(r, c)[0] == -1.0f) || (keyimg_.at<cv::Vec2f>(r, c)[1] == -1.0f))
//                continue;
//            labels.at<Vec2f>(r, c)[0] = keyimg_.at<cv::Vec2f>(r, c)[0];
//            labels.at<Vec2f>(r, c)[1] = keyimg_.at<cv::Vec2f>(r, c)[1];
////                    labels.at<float>(r, c) = keyimg_.at<cv::Vec2f>(r, c)[0];
//        }
//    }

    labels_ = labels.clone();

}

//void GMC::calculateClusters() {labels_
//
//}

void GMC::viewGridResults(Mat& input_img, Mat& output_img) {
    output_img = input_img.clone();
//    cv::cvtColor(output_img, output_img, CV_GRAY2BGR);
    for(int y = 0; y < output_img.rows; ++y){
        for(int x = 0; x < output_img.cols; ++x){
            float lbl = keyimg_.at<Vec2f>(y, x)[0];
            float lbl2 = keyimg_.at<Vec2f>(y, x)[1];
            if(lbl != -1.0f) {
//                int color = min(lbl * 255 / n2 + 50, 255);
//                cout << color << endl;
                cv::circle(output_img, Point(x, y), 2, cv::Scalar(0, 0, 255), 2);
//                float dist_ = disimg_.at<float>(y, x);
                if((abs(lbl) > 1) || ((abs(lbl2) > 1))){
                    std::string output_txt =
                            "x: " + std::to_string(lbl) + "\n z: " + std::to_string(lbl2);
                    cv::putText(output_img, output_txt, Point(x - 3, y - 3), CV_FONT_HERSHEY_PLAIN, 0.8,
                                CV_RGB(0, 250, 0));
                }
            }

        }
    }
}

float GMC::dist3d(Point& p1, Vec3f& p1_dis, Point& p2, Vec3f& p2_dis) {
    float d_residx = p1_dis[0] - p2_dis[0];
    float d_residy = p1_dis[1] - p2_dis[1];
    float d_residz = p1_dis[2] - p2_dis[2];
    float d_resid = sqrt(d_residx * d_residx + d_residy * d_residy + d_residz * d_residz);
    return d_resid;
//    float norm_1 = cv::norm(p1_dis);
//    float norm_2 = cv::norm(p2_dis);
//    float d_resid2 = p1_dis.dot(p2_dis);

//    return d_resid/(norm_1 * norm_2);

//    float dx = p1.x - p2.x;
//    float dy = p1.y - p2.y;
//
//    float d_xy = sqrtf(dx*dx + dy*dy);
//
//    return d_resid + m_/S_ * d_xy;
}

float GMC::dist(Point &p1, Vec2f &p1_dis, Point &p2, Vec2f &p2_dis) {
    float d_residx = p1_dis[0] - p2_dis[0];
    float d_residy = p1_dis[1] - p2_dis[1];
//    float d_resid = d_residx * d_residx + d_residy * d_residy;
    float norm_1 = cv::norm(p1_dis);
    float norm_2 = cv::norm(p2_dis);
    float d_resid = p1_dis.dot(p2_dis);

    return d_resid/(norm_1 * norm_2);
}

