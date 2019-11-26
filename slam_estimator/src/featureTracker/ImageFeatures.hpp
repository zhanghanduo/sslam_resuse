
#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include "../utility/Hash2D.hpp"

class ImageFeatures
{
  public:

    ImageFeatures(const cv::Size& image_size,
      const std::vector<cv::KeyPoint>& keyPoints,
      const cv::Mat& descriptors,
      size_t MatchingCellSize);

    ImageFeatures(const ImageFeatures& imageFeatures);

    std::list<std::pair<size_t, size_t> > FindMatches(
      const std::vector<cv::Point2d>& featurePredictions,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      double matchingDistanceThreshold,
      size_t matchingNeighborhoodThreshold
    ) const;

    inline cv::Mat GetDescriptors() const
    { return descriptors_; }

    inline cv::Mat GetDescriptor( size_t index ) const
    { return descriptors_.row( index ); }

    inline const std::vector<cv::KeyPoint>& GetKeypoints() const
    { return keyPoints_;  }

    inline const cv::KeyPoint& GetKeypoint( size_t index ) const
    { return keyPoints_[ index ]; }

    inline void SetMatchedKeyPoint( size_t index ) const
    { matchedKeyPoints_[ index ] = true; }

    void GetUnmatchedKeyPoints(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors, std::vector<size_t>& indexes) const;

  private:

    mutable std::vector<bool> matchedKeyPoints_;

    cv::Mat descriptors_;

    std::vector<cv::KeyPoint> keyPoints_;

    Hash2D<size_t> hashed_indexes_;

    cv::Size image_size_;

    // we need this because the matching radius parameter is given in hash-cell units.
    size_t hash_cell_size_;

  // helper functions

    typedef std::pair<int, int> iPair;

    iPair GetHash(const cv::Point2d& key, size_t cellSize) const;

    int FindMatch(
      const cv::Point2d& prediction,
      const cv::Mat& descriptor,
      const cv::DescriptorMatcher& descriptorMatcher,
      double matchingDistanceThreshold,
      size_t matchingNeighborhoodThreshold
    ) const;
};
