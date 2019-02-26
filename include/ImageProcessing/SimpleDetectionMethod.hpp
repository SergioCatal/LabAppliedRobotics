#ifndef SIMPLE_DETECTION_METHOD_HPP
#define SIMPLE_DETECTION_METHOD_HPP
#include"DetectionMethod.hpp"
#include"Filter.hpp"
#include"MultiMorphology.hpp"
#include<unordered_map>

class SimpleDetectionMethod : public DetectionMethod {
public:

  /**
   * Constructor that determines the necessary parameters to perform the detection from a JSON file.
   *
   * @param json Json object from which to take method information
   */
  SimpleDetectionMethod(const json11::Json json);

  /**
   * Overridden method to detect a black board in the input image.
   *
   * @param img Image to process
   */
  void detect(cv::Mat & img, int bw, int bh) const override;

  /**
   * Overridden method to read class members from a Json object
   *
   * @param json Json object from which to take kernel information
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the member variables of this object
   */
  json11::Json toJSON() const override;

private:
  void findBoardContours(const cv::Mat &mask, std::vector<cv::Point> &src_pts) const;
  InRangeFilter black_filter;
  MultiMorphology processing;
  int contours_approximation_threshold, min_board_area;
  //static const std::unordered_map<std::string,std::function<void(const json11::Json &, SimpleDetectionMethod*)>> required_parameters;
};

#endif
