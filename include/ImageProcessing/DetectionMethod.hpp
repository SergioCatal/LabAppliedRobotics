#ifndef DETECTION_METHOD_HPP
#define DETECTION_METHOD_HPP
#include "Algorithm.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#define BOARD_N_VERTICES 4

//template<typename T>
class DetectionMethod : public LaR::Algorithm {
public:
    virtual void detect(cv::Mat & img, int board_width, int board_height) const = 0;
    /*{
      static_cast<T*>(this)->detect();
    }*/
};


#endif
