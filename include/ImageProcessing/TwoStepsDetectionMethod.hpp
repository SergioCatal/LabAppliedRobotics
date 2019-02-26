#ifndef TWO_STEPS_DETECTION_METHOD_HPP
#define TWO_STEPS_DETECTION_METHOD_HPP
#include "DetectionMethod.hpp"
#include "Filter.hpp"
#include "MultiMorphology.hpp"

class TwoStepsDetectionMethod : public DetectionMethod{
public:
  /**
   * Constructor that determines the necessary parameters to perform the detection from a JSON file.
   *
   * @param json Json object from which to take method information
   */
  TwoStepsDetectionMethod(const json11::Json json);

  /**
   * Overridden method to detect a black board in the input image.
   *
   * @param img Image to process
   */
  void detect(cv::Mat & img, int board_width, int board_height) const override;

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
  InRangeFilter black_filter_1, black_filter_2;
  MultiMorphology processing_1, processing_2;
  int contours_approximation_threshold_1, contours_approximation_threshold_2;
  int board_first_offset, board_first_expansion, min_area_second_perspective;

  void findBoardContours(const cv::Mat &mask, std::vector<cv::Point> &src_pts, bool second) const;

};
#endif
