/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include <Eigen/Eigen>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// OpenGV
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

//#include "../Map.hpp"
//#include "StereoMatcher.hpp"

/* Stereo Descriptor Match: Represents a feature visible by all 4 frames.
 * 4-way match between two stereo frames
 * frames 1 and 2 correspond to the first stereo frame
 * frames 3 and 4 correspond to the second stereo frame*/
struct SDMatch{

    SDMatch(const cv::DMatch& m1vs2_, const cv::DMatch& m3vs4_, const cv::DMatch& m1vs3_, const cv::DMatch& m2vs4_)
            : m1vs2(m1vs2_), m3vs4(m3vs4_), m1vs3(m1vs3_), m2vs4(m2vs4_) {}

    cv::DMatch m1vs2;

    cv::DMatch m3vs4;

    cv::DMatch m1vs3;

    cv::DMatch m2vs4;
};

/* Absolute pose estimator that uses minimal case algorithms to discard outliers,
 * generic case algorithms for estimation over inliers and non-lineal
 * optimization for estimation improvement */
class PoseEstimator
{
  public:
    enum class PETYPE {CENTRAL, NONCENTRAL};
    enum class MINIMAL_ALGORITHM {KNEIP, // central, P3P case
                                  GAO,  // central, P3P case
                                  EPNP, // central, P6P and PNP case
                                  GP3P, // noncentral, P3P case
                                  };
    enum class GENERIC_ALGORITHM {NONE, // default
                                  EPNP, // central, P6P and PNP case
                                  GPNP, // noncentral, GPNP case
                                  UPNP, // central and noncentral, P3P and PNP case
                                 };

    PoseEstimator(const PETYPE& = PETYPE::CENTRAL, const MINIMAL_ALGORITHM& = MINIMAL_ALGORITHM::KNEIP,
                  const GENERIC_ALGORITHM& = GENERIC_ALGORITHM::NONE, const bool& = false);

    void setRansacThreshold(const double&);
    void setRansacPixelThreshold(const double& pixels, const double& focal_length);
    void setRansacIterations(const int&);

    /* Pose estimation of targetFrame */
    size_t estimatePose(const sptam::Map::SharedKeyFrame&, const sptam::Map::SharedKeyFrame&, const std::vector<SDMatch>&, cv::Matx44d&);

    size_t estimatePose(const sptam::Map::SharedKeyFrame&, const sptam::Map::SharedKeyFrame&,
                        const std::vector<cv::KeyPoint>&, const std::vector<cv::KeyPoint>&,
                        const std::vector<cv::KeyPoint>&, const std::vector<cv::KeyPoint>&,
                        const std::vector<SDMatch>&, cv::Matx44d&);

    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    PETYPE pe_type;
    MINIMAL_ALGORITHM minimal_method;
    GENERIC_ALGORITHM generic_method;
    bool nonlinear_optimization;

  private:

    size_t centralPoseEstimation(const sptam::Map::SharedKeyFrame&, const sptam::Map::SharedKeyFrame&, const std::vector<cv::KeyPoint> &kps1, const std::vector<cv::KeyPoint> &kps2, const std::vector<cv::KeyPoint> &kps3, const std::vector<SDMatch>&, cv::Matx44d&);

    opengv::points_t triangulateReferencePoints(const sptam::Map::SharedKeyFrame&, const std::vector<cv::KeyPoint>&, const std::vector<cv::KeyPoint>&, const std::vector<SDMatch>&);
};

#endif //__POSE_ESTIMATOR_HPP__
