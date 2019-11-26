
#include "opencv_parsers.hpp"

#ifdef SHOW_PROFILING
  #include "log/Logger.hpp"
#endif

template<typename T>
T readParameter(ros::NodeHandle& nh, const std::string &base_name, const std::string &parameterName, const T& defaultValue)
{
  T value;
  nh.param(base_name + "/" + parameterName, value, defaultValue);
  std::cout << "  " << parameterName << ": " << value << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   " + parameterName + ": " + std::to_string( value ) + "\n" );
  #endif

  return value;
}

cv::Ptr<cv::Feature2D>loadGFTT(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters

  int nfeatures;
  nfeatures = readParameter<int>(nh, base_name, "nfeatures", 2000);

  double qualityLevel;
  qualityLevel = readParameter<double>(nh, base_name, "qualityLevel", 0.01);


  double minDistance;
  minDistance = readParameter<double>(nh, base_name, "minDistance", 15.0);


  int blockSize;
  blockSize = readParameter<int>(nh, base_name, "blockSize", 3);


  bool useHarrisDetector;
  useHarrisDetector = readParameter<bool>(nh, base_name, "useHarrisDetector", false);


  double k;
  k = readParameter<double>(nh, base_name, "k", 0.04);

  // create detector
  return cv::GFTTDetector::create(nfeatures, qualityLevel, minDistance, blockSize, useHarrisDetector, k);
}

cv::Ptr<cv::Feature2D>loadORB(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int nFeatures;
  nFeatures = readParameter<int>(nh, base_name, "nFeatures", 500);


  float scaleFactor;
  scaleFactor = readParameter<float>(nh, base_name, "scaleFactor", 1.2f);

  int nLevels;
  nLevels = readParameter<int>(nh, base_name, "nLevels", 8);


  int edgeThreshold;
  edgeThreshold = readParameter<int>(nh, base_name, "edgeThreshold", 31);


  int firstLevel;
  firstLevel = readParameter<int>(nh, base_name, "firstLevel", 0);


  int WTA_K;
  WTA_K = readParameter<int>(nh, base_name, "WTA_K", 2);


  int scoreType;
  scoreType = readParameter<int>(nh, base_name, "scoreType", static_cast<int>(cv::ORB::HARRIS_SCORE));


  int patchSize;
  patchSize = readParameter<int>(nh, base_name, "patchSize", 31);


  int fastThreshold;
  fastThreshold = readParameter<int>(nh, base_name, "fastThreshold", 20);


  return cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
}

cv::Ptr<cv::Feature2D>loadSTAR(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int maxSize;
  maxSize = readParameter<int>(nh, base_name, "maxSize", 45);


  int responseThreshold;
  responseThreshold = readParameter<int>(nh, base_name, "responseThreshold", 30);


  int lineThresholdProjected;
  lineThresholdProjected = readParameter<int>(nh, base_name, "lineThresholdProjected", 10);


  int lineThresholdBinarized;
  lineThresholdBinarized = readParameter<int>(nh, base_name, "lineThresholdBinarized", 8);


  int suppressNonmaxSize;
  suppressNonmaxSize = readParameter<int>(nh, base_name, "suppressNonmaxSize", 5);


  return cv::xfeatures2d::StarDetector::create(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);
}

cv::Ptr<cv::Feature2D>loadSURF(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  float hessianThreshold;
  hessianThreshold = readParameter<float>(nh, base_name, "hessianThreshold", 100.0f);


  int nOctaves;
  nOctaves = readParameter<int>(nh, base_name, "nOctaves", 4);


  int nOctaveLayers;
  nOctaveLayers = readParameter<int>(nh, base_name, "nOctaveLayers", 3);


  bool extended;
  extended = readParameter<bool>(nh, base_name, "extended", false);



  bool upright;
  upright = readParameter<bool>(nh, base_name, "upright", false);


  return cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
}

cv::Ptr<cv::Feature2D>loadFAST(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int threshold;
  threshold = readParameter<int>(nh, base_name, "threshold", 10);


  bool nonmaxSuppression;
  nonmaxSuppression = readParameter<bool>(nh, base_name, "nonmaxSuppression", true);


  int type;
  type = readParameter<int>(nh, base_name, "type", static_cast<int>(cv::FastFeatureDetector::TYPE_9_16));


  return cv::FastFeatureDetector::create(threshold, nonmaxSuppression, type);
}

cv::Ptr<cv::Feature2D>loadAGAST(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int threshold;
  threshold = readParameter<int>(nh, base_name, "threshold", 10);


  bool nonmaxSuppression;
  nonmaxSuppression = readParameter<bool>(nh, base_name, "nonmaxSuppression", true);


  int type;
  type = readParameter<int>(nh, base_name, "type", static_cast<int>(cv::FastFeatureDetector::TYPE_9_16));


  return cv::AgastFeatureDetector::create(threshold, nonmaxSuppression, type);
}

cv::Ptr<cv::Feature2D>loadAKAZE(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int descriptor_type;
  descriptor_type = readParameter<int>(nh, base_name, "descriptor_type", static_cast<int>(cv::AKAZE::DESCRIPTOR_MLDB));


  int descriptor_size;
  descriptor_size = readParameter<int>(nh, base_name, "descriptor_size", 0);


  int descriptor_channels;
  descriptor_channels = readParameter<int>(nh, base_name, "descriptor_channels", 3);


  float threshold;
  threshold = readParameter<float>(nh, base_name, "threshold", 0.001f);


  int nOctaves;
  nOctaves = readParameter<int>(nh, base_name, "nOctaves", 4);


  int nOctaveLayers;
  nOctaveLayers = readParameter<int>(nh, base_name, "nOctaveLayers", 4);


  int diffusivity;
  diffusivity = readParameter<int>(nh, base_name, "diffusivity", static_cast<int>(cv::KAZE::DIFF_PM_G2));


  return cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
}

cv::Ptr<cv::Feature2D>loadBRIEF(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int bytes;
  bytes = readParameter<int>(nh, base_name, "bytes", 32);


  bool use_orientation;
  use_orientation = readParameter<bool>(nh, base_name, "use_orientation", false);


  // create detector
  return cv::xfeatures2d::BriefDescriptorExtractor::create( bytes, use_orientation );
}

cv::Ptr<cv::Feature2D>loadBRISK(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int thresh;
  thresh = readParameter<int>(nh, base_name, "thresh", 30);


  int octaves;
  octaves = readParameter<int>(nh, base_name, "octaves", 3);


  float patternScale;
  patternScale = readParameter<float>(nh, base_name, "patternScale", 1.0f);


  return cv::BRISK::create(thresh, octaves, patternScale);
}

cv::Ptr<cv::Feature2D>loadLATCH(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int bytes;
  bytes = readParameter<int>(nh, base_name, "bytes", 32);


  bool rotationInvariance;
  rotationInvariance = readParameter<bool>(nh, base_name, "rotationInvariance", true);


  int half_ssd_size;
  half_ssd_size = readParameter<int>(nh, base_name, "half_ssd_size", 3);


  return cv::xfeatures2d::LATCH::create(bytes, rotationInvariance, half_ssd_size);
}

cv::Ptr<cv::Feature2D>loadLUCID(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  int lucid_kernel;
  lucid_kernel = readParameter<int>(nh, base_name, "lucid_kernel", 1);


  int blur_kernel;
  blur_kernel = readParameter<int>(nh, base_name, "blur_kernel", 1);


  return cv::xfeatures2d::LUCID::create(lucid_kernel, blur_kernel);
}

cv::Ptr<cv::Feature2D>loadFREAK(ros::NodeHandle& nh, const std::string& base_name) {

  // load parameters
  bool orientationNormalized;
  orientationNormalized = readParameter<bool>(nh, base_name, "orientationNormalized", true);


  bool scaleNormalized;
  scaleNormalized = readParameter<bool>(nh, base_name, "scaleNormalized", true);


  float patternScale;
  patternScale = readParameter<float>(nh, base_name, "patternScale", 22.0f);


  int nOctaves;
  nOctaves = readParameter<int>(nh, base_name, "nOctaves", 4);


  const std::vector<int>& selectedPairs = std::vector<int>();

  return cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs);
}

cv::Ptr<cv::FeatureDetector> loadFeatureDetector(ros::NodeHandle& nh, const std::string& algorithm, const std::string& base_name)
{
  cv::Ptr<cv::FeatureDetector> detector;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   detector: " + algorithm + "\n" );
  #endif


  if (algorithm == "GFTT") {
    detector = loadGFTT(nh, base_name);
  }
  else if (algorithm == "ORB") {
    detector = loadORB(nh, base_name);
  }
  else if (algorithm == "STAR") {
    detector = loadSTAR(nh, base_name);
  }
  else if (algorithm == "BRISK") {
    detector = loadBRISK(nh, base_name);
  }
  else if (algorithm == "FAST") {
    detector = loadFAST(nh, base_name);
  }
  else if (algorithm == "AGAST") {
    detector = loadAGAST(nh, base_name);
  }
  else if (algorithm == "AKAZE") {
    detector = loadAKAZE(nh, base_name);
  }
  else if (algorithm == "SURF") {
    detector = loadSURF(nh, base_name);
  }
  else throw std::runtime_error("Requested feature detector algorithm not yet supported in factory method");

  return detector;
}

cv::Ptr<cv::DescriptorExtractor> loadDescriptorExtractor(ros::NodeHandle& nh, const std::string& algorithm, const std::string& base_name)
{
  cv::Ptr<cv::DescriptorExtractor> extractor;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   descriptor: " + algorithm + "\n" );
  #endif

  if (algorithm == "BRIEF") {
    extractor = loadBRIEF(nh, base_name);
  }
  else if (algorithm == "BRISK") {
    extractor = loadBRISK(nh, base_name);
  }
  else if (algorithm == "ORB") {
    extractor = loadORB(nh, base_name);
  }
  else if (algorithm ==  "FREAK") {
    extractor = loadFREAK( nh, base_name );
  }
  else if (algorithm == "LATCH") {
    extractor = loadLATCH( nh, base_name );
  }
  else if (algorithm == "LUCID") {
    extractor = loadLUCID( nh, base_name );
  }
  else if (algorithm == "AKAZE") {
    extractor = loadAKAZE(nh, base_name);
  }
  else if (algorithm == "SURF") {
    extractor = loadSURF(nh, base_name);
  }
  else throw std::runtime_error("Requested descriptor extractor algorithm not yet supported in factory method");

  return extractor;
}

cv::Ptr<cv::DescriptorMatcher> loadDescriptorMatcher(ros::NodeHandle& nh, const std::string& algorithm, const std::string& base_name)
{
  cv::Ptr<cv::DescriptorMatcher> matcher;

  bool crossCheck;
  crossCheck = readParameter<bool>(nh, base_name, "crossCheck", false);


  if (algorithm == "BruteForce-Hamming") {
    matcher = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING, crossCheck);
  }
  else if (algorithm == "BruteForce") {
    matcher = cv::makePtr<cv::BFMatcher>(cv::NORM_L2, crossCheck);
  }
  else if (algorithm == "BruteForce-L1") {
    matcher = cv::makePtr<cv::BFMatcher>(cv::NORM_L1, crossCheck);
  }
  else if (algorithm == "BruteForce-Hamming(2)") {
    matcher = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING2, crossCheck);
  }
  else throw std::runtime_error("Requested descriptor matcher algorithm not yet supported in factory method");

  return matcher;
}

#endif
