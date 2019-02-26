#ifndef RECOGNITOR_H
#define RECOGNITOR_H
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include"json.hpp"
#include"Victim.hpp"
#include"Obstacle.hpp"
#include"ContoursFinder.hpp"
#include"Filter.hpp"
#include"MultiMorphology.hpp"
#include"Map.hpp"
namespace LaR{

  class Recognitor : JSONCompatible{
  public:
    Recognitor(const json11::Json &json);
    void recognize(const cv::Mat &img, Map &m);
    void fromJSON(const json11::Json &json) override;
    json11::Json toJSON() const override;

  private:
    InRangeFilter blue_filter, green_filter, black_green_filter;
    TwoRangesFilter red_filter;
    MultiMorphology blue_processing, green_processing, black_green_processing, red_processing;
    ContoursFinder<Obstacle> gate_contour_finder;
    ContoursFinder<std::vector<Obstacle> > obstacles_contour_finder;
    ContoursFinder<std::vector<Victim>> victims_contour_finder;

    int min_circle_area, min_digit_area, digit_img_size;

    /**
     * Method that finds the red obstacles in the image
     *
     * @param img: image to analyze in order to find the red obstacles
     * @param obstacles: an empty vector of Obstacle objects in which to insert all the obstacles found in the image
     */
    void getObstacles(const cv::Mat &img, std::vector<Obstacle> &obstacles) const;

    /**
     * Method that finds the blue gate in the image
     *
     * @param img: image to analyze in order to find the gate
     * @param obstacle: an Obstacle object in which to insert the goal
     */
    void getGate(const cv::Mat &img, Obstacle &gate) const;

    /**
     * Method that finds the green circles with the digit in the image, i.e. the Victims
     *
     * @param img: image to analyze in order to find the victims. It is not const because this is the last operation
     * and it can be performed directly on the hsv_img
     * @param victims: an empty vector of Victim objects in which to insert all the victims found in the image
     */
     void getCircles(const cv::Mat &original, const cv::Mat &img, std::vector<Victim> &victims) const;
  };

};

#endif
