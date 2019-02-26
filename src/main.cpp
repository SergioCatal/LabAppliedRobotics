#include <iostream>
#include <sstream>

#include <opencv2/core.hpp>

#include "robot_project.h"
#include "path.h"

#include <sys/types.h>
#include <dirent.h>


int main(int argc, char* argv[]) {

  Mat img = imread("./data/test_imgs/2018-12-20-123545.jpg");
  imshow("Original",img);
  cv::waitKey(0);

  RobotProject rp(argc, argv);

  rp.preprocessMap(img);
  Path p;
  rp.planPath(img, p);

}