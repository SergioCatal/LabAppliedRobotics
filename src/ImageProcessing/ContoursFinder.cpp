#include"ContoursFinder.hpp"
#include <exception>
void find_corners(const cv::Mat &img, std::vector<cv::Point> &circles){
    int img_width = img.cols;
    int img_height = img.rows;
    int x_offset = img_width*0.1;
    int y_offset = img_height*0.01;
    int x_section = img_width /6;
    int y_section = img_height /5;


    int parameter1 = 255;
    int parameter2 = ADAPTIVE_THRESH_MEAN_C;
    int parameter3 = THRESH_BINARY;
    int parameter4 = 5;
    int parameter5 = 7;
    cv::Mat imgtl, imgtr, imgbl, imgbr;
    std::vector<cv::Vec3f> ctl, ctr, cbl, cbr;

    imgtl = img(cv::Rect(x_offset, y_offset, x_section, y_section));
    cv::cvtColor(imgtl, imgtl, CV_BGR2GRAY);
    cv::adaptiveThreshold(imgtl,imgtl,parameter1, parameter2, parameter3, parameter4 , parameter5);
    cv::HoughCircles(imgtl, ctl, CV_HOUGH_GRADIENT, 1.5, 500, 150, 60, 10, 35);
    //cv::imshow("TOP LEFT",imgtl);


    imgtr = img(cv::Rect(img_width-x_offset-x_section, y_offset, x_section, y_section));
    cv::cvtColor(imgtr,imgtr,CV_BGR2GRAY);
    cv::adaptiveThreshold(imgtr,imgtr,parameter1, parameter2, parameter3, parameter4 , parameter5);
    cv::HoughCircles(imgtr, ctr, CV_HOUGH_GRADIENT, 1.5, 500, 150, 60, 10, 35);
    //cv::imshow("TOP RIGHT",imgtr);


    imgbl = img(cv::Rect(x_offset, img_height-y_offset-y_section, x_section, y_section));
    cv::cvtColor(imgbl,imgbl,CV_BGR2GRAY);
    cv::adaptiveThreshold(imgbl,imgbl,parameter1, parameter2, parameter3, parameter4 , parameter5);
    cv::HoughCircles(imgbl, cbl, CV_HOUGH_GRADIENT, 1.5, 500, 150, 60, 10, 35);
    //cv::imshow("BOTTOM LEFT",imgbl);


    imgbr = img(cv::Rect(img_width-x_offset-x_section, img_height-y_offset-y_section, x_section, y_section));
    cv::cvtColor(imgbr,imgbr,CV_BGR2GRAY);
    cv::adaptiveThreshold(imgbr,imgbr,parameter1, parameter2, parameter3, parameter4 , parameter5);
    cv::HoughCircles(imgbr, cbr, CV_HOUGH_GRADIENT, 1.5, 500, 150, 60, 10, 35);
    //imshow("BOTTOM RIGHT",imgbr);
    //cv::waitKey(0);


    if(ctl.size() >0) {
        ctl[0][0] += x_offset;
        ctl[0][1] += y_offset;
        circles.push_back(cv::Point(ctl[0][0],ctl[0][1]));
    }

    if(ctr.size() > 0) {
        ctr[0][0] += img_width-x_offset-x_section;
        ctr[0][1] += y_offset;
        circles.push_back(cv::Point(ctr[0][0],ctr[0][1]));
    }

    if(cbl.size() > 0) {
        cbl[0][0] += x_offset;
        cbl[0][1] += img_height-y_offset-y_section;
        circles.push_back(cv::Point(cbl[0][0],cbl[0][1]));
    }

    if(cbr.size() > 0) {
        cbr[0][0] += img_width-x_offset-x_section;
        cbr[0][1] += img_height-y_offset-y_section;
        circles.push_back(cv::Point(cbr[0][0],cbr[0][1]));
    }

    if(circles.size() != 4)
      throw std::logic_error("Invalid number of corners detected");

    #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::Mat img_copy = img.clone();
      for(unsigned int i = 0;i < circles.size();i++ ) {
          cv::circle(img_copy,circles[i],4,cv::Scalar(0,0,255),4,LINE_AA,0);
      }
      imshow("CIRCLES",img_copy);
      waitKey(0);
    #endif
}


template <>
void ContoursFinder<std::vector<Obstacle>>::evaluateContour(std::vector<Obstacle> &obstacles, std::vector<Point> &contour)const{
  if(contour.size() > 2 && contour.size() < 12 && (cv::contourArea(contour) > 1000)){
    cv::Moments mu = moments(contour, false);
    cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);
    std::vector<cv::Point2f> vertices;
    for(int i = 0; i < contour.size(); i++)
      vertices.push_back(cv::Point2f(contour[i].x, contour[i].y));

    obstacles.push_back(Obstacle(center, vertices));
  }
}

template <>
void ContoursFinder<Obstacle>::evaluateContour(Obstacle &obstacle, std::vector<Point> &contour)const{
  if(contour.size() == 4){
    if(obstacle.getNVertices() > 0 && cv::contourArea(contour) <= obstacle.getArea())
      return;

    cv::Moments mu = moments(contour, false);
    cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);

    std::vector<cv::Point2f> vertices;
    for(int i = 0; i < contour.size(); i++)
      vertices.push_back(cv::Point2f(contour[i].x, contour[i].y));

    obstacle = Obstacle(center, vertices);
  }
}

template <>
void ContoursFinder<std::vector<Victim>>::findContours(const cv::Mat &img, std::vector<Victim>  &victims) const{
  std::vector<cv::Vec3f> ctl;
  cv::HoughCircles(img, ctl, CV_HOUGH_GRADIENT, 1.5, 50, 255, 21, 80, 95);

  for(int i = 0; i < ctl.size(); i++)
    victims.push_back(Victim(-1, cv::Point2f(ctl[i][0], ctl[i][1]), ctl[i][2]));
}

//The Json for this will have 6 parameters: dp, max_dist, threshold, min_circle_accumulation_value, min_radius, max_radius
template <>
void ContoursFinder<std::vector<cv::Point>>::findContours(const cv::Mat &img, std::vector<cv::Point>  &corners) const{
  find_corners(img, corners);

  /*
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if(options.size() != 4){
    throw "NOT enough parameters for HOUGH TRANSFORM";
  }

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(img, circles, CV_HOUGH_GRADIENT, options[0], approximation_threshold, min_area, options[1], options[2], options[3]);

  for(int k = 0; k < circles.size(); k++){
    corners.push_back(cv::Point(circles[k][0], circles[k][1]));
    std::cout << circles[k][0] << " " << circles[k][1] << " " << circles[k][2] << std::endl;
  }
*/

}


/*
int main(){
  std::string err;
  std::ifstream f("input/file.json");
  std::stringstream buf;
  buf << f.rdbuf();
  std::string json_string = buf.str();
  json11::Json json = json11::Json::parse(json_string, err);

  cv::Mat img = cv::imread("test_image.jpg");
  cv::inRange(img, cv::Scalar(0,0,0), cv::Scalar(100,100,100), img);

  Obstacle ob;
  Recognitor r(JSONCompatible::element_of(json, "ContourFinder"));
  r.recognize(img);
  cv::imshow("BLOH", img);
  cv::waitKey(0);

  return 0;
}*/
