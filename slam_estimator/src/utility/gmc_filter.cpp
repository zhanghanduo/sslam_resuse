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
        ny = 10;
        dx = w / float(nx);
        dy = h / float(ny);
        S_ = (dx + dy + 1)/2;   // default window size
    }
    else
        S_ = S;

    assignGrids();
//    calculateClusters();

}

void GMC::assignGrids() {
    // cluster candidate number
//    int n = nx * ny;
    cv::Size cell_size(static_cast<int>(ceilf( w / nx )),
                       static_cast<int>(ceilf( h / ny )));

    std::vector<Point> keypoints_list;

    for (int xi = 0; xi < nx; xi++) {
        for (int yj = 0; yj < ny; yj++) {
//            this->centers.push_back( Point2f(j*dx+dx/2, i*dy+dy/2));
            // 1. Build grids
            cv::Rect cell_rect(xi * cell_size.width, yj * cell_size.height, cell_size.width, cell_size.height);
            if (cell_rect.tl().x + cell_rect.width > keyimg_.size().width) cell_rect.width = (keyimg_.size().width - cell_rect.tl().x);
            if (cell_rect.tl().y + cell_rect.height > keyimg_.size().height) cell_rect.height = (keyimg_.size().height - cell_rect.tl().y);
            cv::Mat cell_image = keyimg_(cell_rect);
            int sum = 0;

            // 2. Check how many key points in this grid
            std::vector<cv::Point> cell_keypoints;
            std::vector<cv::Vec3f> cell_dists;
            for(int i = 0; i < cell_image.rows; i++) {
                for(int j = 0; j < cell_image.cols; j++) {
                    cv::Vec3f pixel_dis = cell_image.at<cv::Vec3f>(i, j);
                    if(pixel_dis[0] != -1.0f) {
                        sum ++;
                        // compensate offset position
                        cell_keypoints.emplace_back(Point(j + cell_rect.tl().x, i + cell_rect.tl().y));
                        cell_dists.push_back(pixel_dis);
                    }
                }
            }

            // If there is no features or only one feature, do nothing.
            if(sum < 4)
                continue;

            // 3. Randomly select a point as the center of this local cluster assignment
            int sum2 = sum / 2;
            centers.push_back(cell_keypoints[sum2]);
            dist_centers.push_back(cell_dists[sum2]);
        }
    }

    n2 = centers.size();
    cout << "center number" << n2 << endl;

    // 4. Initialize labels and distance maps
    std::vector<int> label_vec(n2);
    for (int i = 0; i < n2; i++)
        label_vec[i] = i;

    Mat labels = -1 * Mat::ones(this->keyimg_.size(), CV_32S);
    Mat dists = -1 * Mat::ones(this->keyimg_.size(), CV_32F);
    Mat window;
    cv::Point p1, p2;
    cv::Vec3f p1_dist, p2_dist;

    // Iterate 10 times to converge
    for (int i = 0; i < 5; i++) {
        // For each center
        for (int c = 0; c < n2; c++) {
            int label = label_vec[c];
            p1 = centers[c];
            p1_dist = dist_centers[c];
            int xmin = max<int>(p1.x - S_, 0);
            int ymin = max<int>(p1.y - S_, 0);
            int xmax = min<int>(p1.x + S_, w - 1);
            int ymax = min<int>(p1.y + S_, h - 1);

            // Search in a window around the center
            window = keyimg_(Range(ymin, ymax), Range(xmin, xmax));

            // Reassign pixels to nearest center
            for (int r = 0; r < window.rows; r++) {
                for (int j = 0; j < window.cols; j++) {
                    cv::Vec3f pixel_dis = window.at<cv::Vec3f>(r, j);
                    if(pixel_dis[0] != -1.0f) {
                        p2 = Point2i(xmin + j, ymin + r);
                        p2_dist = pixel_dis;
                        float d = dist3d(p1, p1_dist, p2, p2_dist);
//                        if(d > 0.01)
//                            cout << "iteration: " << i << "cluster: " << c << "dist: " << d << endl;
                        float last_d = dists.at<float>(p2);
                        if (d < last_d || last_d == -1) {
                            dists.at<float>(p2) = d;
                            labels.at<int>(p2) = label;
//                            cout << "label: " << label << endl;
                        }
                    }
                }
            }

        }
    }

    // Store labels for each key point
    labels_ = labels.clone();
    disimg_ = dists.clone();
//    labels_ = n2 * labels_ / 255;

//    labels_.convertTo(labels_, CV_32F);
}

void GMC::calculateClusters() {

    cv::Size cell_size(static_cast<int>(ceilf( w / nx )),
                       static_cast<int>(ceilf( h / ny )));

    std::vector<Point> keypoints_list;

    for (int xi = 0; xi < nx; xi++) {
        for (int yj = 0; yj < ny; yj++) {
//            this->centers.push_back( Point2f(j*dx+dx/2, i*dy+dy/2));
            // 1. Build grids
            cv::Rect cell_rect(xi * cell_size.width, yj * cell_size.height, cell_size.width, cell_size.height);
            if (cell_rect.tl().x + cell_rect.width > keyimg_.size().width) cell_rect.width = (keyimg_.size().width - cell_rect.tl().x);
            if (cell_rect.tl().y + cell_rect.height > keyimg_.size().height) cell_rect.height = (keyimg_.size().height - cell_rect.tl().y);
            cv::Mat cell_image = keyimg_(cell_rect);
            int sum = 0;

            // 2. Check how many key points in this grid
            std::vector<cv::Point> cell_keypoints;
            std::vector<cv::Vec3f> cell_dists;
            for(int i = 0; i < cell_image.rows; i++) {
                for(int j = 0; j < cell_image.cols; j++) {
                    cv::Vec3f pixel_dis = cell_image.at<cv::Vec3f>(i, j);
                    if(pixel_dis[0] != -1.0f) {
                        sum ++;
                        // compensate offset position
                        cell_keypoints.emplace_back(Point(j + cell_rect.tl().x, i + cell_rect.tl().y));
                        cell_dists.push_back(pixel_dis);
                    }
                }
            }

            // If there is no features or only one feature, do nothing.
            if(sum < 4)
                continue;

            // 3. Randomly select a point as the center of this local cluster assignment
            int sum2 = sum / 2;
            centers.push_back(cell_keypoints[sum2]);
            dist_centers.push_back(cell_dists[sum2]);
        }
    }

}

void GMC::viewGridResults(Mat& input_img, Mat& output_img) {
    output_img = input_img.clone();
    cv::cvtColor(output_img, output_img, CV_GRAY2BGR);
    for(int y = 0; y < output_img.rows; ++y){
        for(int x = 0; x < output_img.cols; ++x){
            int lbl = labels_.at<int>(y, x);
            if(lbl != -1) {
                int color = min(lbl * 255 / n2 + 50, 255);
//                cout << color << endl;
                cv::circle(output_img, Point(x, y), 2, cv::Scalar(0, 0, color), 2);
                float dist_ = disimg_.at<float>(y, x);
                if(dist_ > 0.5)
                    cv::putText(output_img, std::to_string(dist_), Point(x - 3, y - 3), CV_FONT_HERSHEY_PLAIN, 0.8,
                        CV_RGB(0, 250, 0));
//                output_img.at<Vec3b>(y, x) = cv::Vec3b(0, lbl, (1-lbl));
            }

//            if(num_pixels[lbl])
//                output_img.at<Vec3b>(y, x) = avg_colors[lbl];
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

