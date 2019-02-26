#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP
#include "json.hpp"
#include <opencv2/opencv.hpp>

namespace LaR{
  //template <typename T>
  class Algorithm :  public JSONCompatible {
  public:
    //virtual T run(cv::Mat &img) = 0;
    virtual ~Algorithm(){}
  };
};

#endif
