#ifndef ROBOT_PROJECT
#define ROBOT_PROJECT

#include <opencv2/opencv.hpp>
#include <vector>
#include "path.h"
#include "json.hpp"
#include "Map.hpp"
#include "Detector.hpp"


// Define your own class RobotProject, that implements and exposes the following methods.
// NB: The input images are already undistorted.
class RobotProject : public JSONCompatible
{
public:
  // Constructor taking as argument the command line parameters
  RobotProject(int argc, char * argv[]);

  // Method invoked to preprocess the map (extrinsic calibration + reconstruction of layout)
  bool preprocessMap(cv::Mat const & img);

  // Method invoked when a new path must be planned (detect initial robot position from img)
  bool planPath(cv::Mat const & img, Path & path);

  // Method invoked periodically to determine the position of the robot within the map.
  // The output state is a vector of three elements, x, y and theta.
  bool localize(cv::Mat const & img,
                std::vector<double> & state);

  void fromJSON(const json11::Json &json) override;

  json11::Json toJSON() const override;

private:
  Map map;
  Detector detector;
  float clippingQuantity, victim_bonus, robot_k, robot_velocity;
  int points_per_cell, mission_type, n_of_trials;

  float robot_x, robot_y, robot_theta;
};

#endif
