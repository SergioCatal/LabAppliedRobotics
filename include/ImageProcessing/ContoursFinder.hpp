#ifndef CONTOURS_FINDER_HPP
#define CONTOURS_FINDER_HPP
#include "json.hpp"
#include "Obstacle.hpp"
#include "Victim.hpp"
#include <string>

/**
 * Class used to collect the parameters (from an input Json file) of a contour-finding operation and to perform it in the proper way based on the shapes (as templates) we are looking for
 * Invariants: - Maximum 4 elements in the vector params
 */
template<typename T>
class ContoursFinder : JSONCompatible{
public:
  /**
   * Constructor that takes the required parameters from a Json object
   *
   * @argument json : Json object containing the parameters
   */
  ContoursFinder(const json11::Json &json){
    fromJSON(json);
  }

  /**
   * Constructor that takes the required parameters explicitly
   *
   * @argument app_threshold : threshold for the polygon approximation
   * @argument area : the minimum area of acceptable contours
   */
  ContoursFinder(int app_threshold, int area) : approximation_threshold(app_threshold), min_area(area){
  }

  /**
   * Method that finds contours in the image and stores them in the appropriate object
   *
   * @argument img : the image to process for finding the contours
   * @argument objs : the objects to contain the found shapes
   */
  void findContours(const cv::Mat &img, T &objs) const {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> approx_curve;

    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i)  {
      cv::convexHull(cv::Mat(contours[i]), approx_curve, true, true);
      cv::approxPolyDP(approx_curve, approx_curve, approximation_threshold, true);
      #ifdef PRINT_INTERMEDIATE
        std::cout << "Approximated Polygon: ";
        for(int k = 0; k < approx_curve.size(); k++) std::cout << "(" << approx_curve[k].x << "," << approx_curve[k].y << ") ";
        std::cout << std::endl;
      #endif
      evaluateContour(objs, approx_curve);
    }

  }

  /**
   * Method that alerts the user when this non-specialized method is used. The only acceptable input arguments are the ones for which the method is specialized.
   *
   * @argument obj : the object to contain the found shape
   * @argument contour : the actual vector of points representing the currently evaluated contour
   */
  void evaluateContour(T &obj, std::vector<Point> &contour) const {
    std::cout << "Unaccepted template specialization for ContoursFinder object" << std::endl;
  }

  /**
   * Overridden method to read the parameters of this object from a Json object
   *
   * @argument json : Json object from which to take information about the operations to perform
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override{
    if(json.type() != json11::Json::ARRAY)
        throw JSONCompatible::InvalidJSON("reading ContoursFinder", "not receiving an array");

    auto arr = json.array_items();
    if(arr.size() == 0)
      throw JSONCompatible::InvalidJSON("retrieving ContoursFinder", "invalid number of elements in array");

    if(arr[0].type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("retrieving ContoursFinder", "invalid type in first element");

    approximation_threshold = arr[0].int_value();

    if(arr.size() > 1){
      if(arr[1].type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("retrieving ContoursFinder", "invalid type in second element");

      min_area = arr[1].int_value();

      if(arr.size() <= 6){
        for(unsigned int k = 2; k < arr.size(); k++){
          if(arr[k].type() != json11::Json::NUMBER)
            throw JSONCompatible::InvalidJSON("retrieving ContoursFinder", "invalid type in " + std::to_string(k) + "th element");

          options.push_back(arr[k].number_value());
        }
      } else {
        throw JSONCompatible::InvalidJSON("retrieving ContoursFinder", "too many elements");
      }
    }
  }

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the object parameters
   */
  json11::Json toJSON() const override {
    return json11::Json();
  }

private:
  int approximation_threshold;
  int min_area;
  std::vector<float> options;
};


/**
 * Method that determines if a contour is valid for an obstacle and, if yes, adds it to the vector of obstacles. This is a specialization of the general method.
 *
 * @argument obstacles : a vector of obstacles on which to add the new obstacle
 * @argument contour : the actual vector of points representing the currently evaluated contour
 */
template <>
void ContoursFinder<std::vector<Obstacle>>::evaluateContour(std::vector<Obstacle> &obstacles, std::vector<Point> &contour)const;

/**
 * Method that determines if a contour is valid to be the goal and, if yes, modifies the @obstacle accordingly. This is a specialization of the general method.
 *
 * @argument goal : the object containing the goal
 * @argument contour : the actual vector of points representing the currently evaluated contour
 */
template <>
void ContoursFinder<Obstacle>::evaluateContour(Obstacle &goal, std::vector<Point> &contour)const;

/**
 * Method that determines if a contour is valid to be a victim and, if yes, modifies the vector of victims. This is a specialization of the general method.
 *
 * @argument victims : a vector of obstacles on which to add the new one
 * @argument contour : the actual vector of points representing the currently evaluated contour
 */
template <>
void ContoursFinder<std::vector<Victim>>::evaluateContour(std::vector<Victim> &victims, std::vector<Point> &contour)const;

/**
 * Method that finds the corners of the board and stores them in a std::vector<cv::Vec3f>
 *
 * @argument img : the image to process for finding the contours
 * @argument objs : the objects to contain the found shapes
 */
template <>
void ContoursFinder<std::vector<cv::Point>>::findContours(const cv::Mat &img, std::vector<cv::Point>  &corners) const;

template <>
void ContoursFinder<std::vector<Victim>>::findContours(const cv::Mat &img, std::vector<Victim>  &victims) const;


#endif
