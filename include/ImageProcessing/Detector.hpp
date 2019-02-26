#ifndef DETECTOR_HPP
#define DETECTOR_HPP
#include <opencv2/core.hpp>
#include <vector>

#include "json.hpp"
#include "Filter.hpp"
#include "Obstacle.hpp"
#include "ContoursFinder.hpp"
#include "TwoStepsDetectionMethod.hpp"
#include "SimpleDetectionMethod.hpp"
#include "path.h"

class Detector : public JSONCompatible{
private:
  float board_width, board_height;
  int tape_width;
  InRangeFilter robot_filter, corners_filter;
  MultiMorphology robot_processing, corners_processing;
  SimpleDetectionMethod detection_method;
  ContoursFinder<Obstacle> robot_contour_finder;
  ContoursFinder<std::vector<cv::Point>> corners_contour_finder;
  cv::Mat robot_plane_homography;


public:
  Detector(json11::Json json = json11::Json());
  bool localize(const cv::Mat &img, std::vector<double> & state);
  void detect_board_and_warp(cv::Mat &img);
  void detect_robot_plane(const cv::Mat &img);
  void fromJSON(const json11::Json &json) override;
  json11::Json toJSON() const override;
};

#endif
