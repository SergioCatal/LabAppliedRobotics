#include "SimpleDetectionMethod.hpp"
#include<iostream>
#include <exception> //DEBUG

//#define SHOW_INTERMEDIATE_DETECTION
/*const std::unordered_map<std::string,std::function<void(const json11::Json &, SimpleDetectionMethod*)>>
                         SimpleDetectionMethod::required_parameters{
                           {"Filter", [](const json11::Json &json, SimpleDetectionMethod* sdm){sdm->black_filter.fromJSON(json);}},
                           {"Processing", [](const json11::Json &json, SimpleDetectionMethod* sdm){sdm->processing.fromJSON(json);}}
                         };
 */

SimpleDetectionMethod::SimpleDetectionMethod(const json11::Json json) :
                        black_filter(JSONCompatible::element_of(json, "Filter")),
                        processing(JSONCompatible::element_of(json, "Processing")){
  fromJSON(json);
}

void SimpleDetectionMethod::findBoardContours(const cv::Mat &mask, std::vector<cv::Point> &src_pts) const{
  // Variables for the contours detection
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> &approx_curve = src_pts;

  // Find the external contours of the black mask
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  // Variables needed to find the biggest rectangular contour by area
  double max_area = min_board_area, second_max_area = 0;
  int biggest_area_position = -1, second_biggest_area_position = -1;

  // Iterate each contour and find the second biggest rectangular contour that is the board
  for (int i=0; i<contours.size(); ++i){
    // fit a closed polygon (with less vertices) to the given contour, with an approximation accuracy of black_contour_approx
    cv::approxPolyDP(contours[i], approx_curve, contours_approximation_threshold, true);

    // Calculate the area of the contour
    double area = cv::contourArea(contours[i]);

    // If this contour is reactangular and have the biggest rectangular area so far
    if(approx_curve.size() >= BOARD_N_VERTICES){
      if(area > max_area) { //TODO: CONSTANT
        // Update the biggest area and the position of this contour
            second_max_area = max_area;
            max_area = area;
            second_biggest_area_position = biggest_area_position;
            biggest_area_position = i;
        } else if(area > second_max_area){
        second_max_area = area;
        second_biggest_area_position = i;
      }
    }
  }


  if(second_biggest_area_position < 0){
    src_pts.clear();
    return;
  }

  std::vector<cv::Point> board_contour_raw = contours[second_biggest_area_position];
  cv::approxPolyDP(board_contour_raw, approx_curve, contours_approximation_threshold, true); //TODO: AVOID RECOMPUTING APPROX

  #ifdef PRINT_INTERMEDIATE
      // Print the coordinates of the board in the image
      for(int i = 0; i < approx_curve.size(); i++) {
        std::cout << approx_curve[i] << " ";
      }
      std::cout << std::endl;
  #endif

  cv::Point2f center((approx_curve[0].x + approx_curve[1].x + approx_curve[2].x + approx_curve[3].x)/4, (approx_curve[0].y + approx_curve[1].y + approx_curve[2].y + approx_curve[3].y)/4);
  class angle_index{
    public: float angle; char index; cv::Point point;
    angle_index(float a, char i, cv::Point& p):angle(a), index(i), point(p){}
    bool operator <(const angle_index& other) const{return angle < other.angle;}};
  std::vector<angle_index> angles;
  for(int ind = 0; ind < 4; ind++) angles.push_back(angle_index(atan2(approx_curve[ind].y - center.y, approx_curve[ind].x - center.x), ind, approx_curve[ind]));
  std::sort(angles.begin( ), angles.end( ));

  // Calculate the distance between each adjacent board point
  float distance_0_to_1 = std::sqrt( std::pow(angles[0].point.x - angles[1].point.x,2) +  std::pow(angles[0].point.y - angles[1].point.y,2) );
  float distance_3_to_0 = std::sqrt( std::pow(angles[3].point.x - angles[0].point.x,2) +  std::pow(angles[3].point.y - angles[0].point.y,2) );

  if(distance_0_to_1 > distance_3_to_0) {
      approx_curve[0] = angles[0].point;
      approx_curve[1] = angles[1].point;
      approx_curve[2] = angles[2].point;
      approx_curve[3] = angles[3].point;
  } else {
      approx_curve[0] = angles[1].point;
      approx_curve[1] = angles[2].point;
      approx_curve[2] = angles[3].point;
      approx_curve[3] = angles[0].point;
  }
}

void SimpleDetectionMethod::detect(cv::Mat & img, int bw, int bh) const {

  std::cout << "Detecting with SimpleDetectionMethod" << std::endl;
  cv::Mat tmp;
  cv::cvtColor(img, tmp, CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(tmp, channels);
  cv::GaussianBlur(channels[0], channels[0], cv::Size(11, 11), 27);
  cv::adaptiveThreshold(channels[0], channels[0], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 299, -4);

  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("FILTER HUE", channels[0]);
      cv::waitKey(0);
  #endif

  cv::GaussianBlur(channels[2], channels[2], cv::Size(3, 3), 1);
  cv::adaptiveThreshold(channels[2], channels[2], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 153, 3);
  cv::bitwise_not(channels[2], channels[2]);
  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("FILTER VAL", channels[2]);
      cv::waitKey(0);
  #endif

  cv::bitwise_and(channels[0], channels[2], tmp);

//  black_filter.apply(tmp);

  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("AfterFilter", tmp);
      cv::waitKey(0);
  #endif

  //cv::adaptiveThreshold(tmp, tmp, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 1);


  processing.apply(tmp);

  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("AfterProcessing", tmp);
      cv::waitKey(0);
  #endif

  std::vector<cv::Point> src_pts;
  findBoardContours(tmp, src_pts);
  if(src_pts.size() != 4)
    throw std::logic_error("Unable to find board");

  #ifdef SHOW_INTERMEDIATE_DETECTION
  {
    cv::Mat tmpImg = img.clone();
    std::vector<std::vector<cv::Point>> contours_approx = {src_pts};
    std::cout << "PTS: " << src_pts.size() << std::endl;
    cv::drawContours(tmpImg, contours_approx, -1, cv::Scalar(255, 65, 65), 2, cv::LINE_AA);
    cv::imshow("CONTOURS2", tmpImg);
    cv::waitKey(0);
  }
  #endif

  cv::Size board_size(bw, bh);
  std::vector<cv::Point> dst_pts = {{0,0}, {board_size.width, 0}, {board_size.width, board_size.height}, {0, board_size.height}};
  cv::Mat h = cv::findHomography(src_pts, dst_pts);
  cv::warpPerspective(img, img, h, board_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

  #if defined SHOW_INTERMEDIATE_DETECTION || defined SHOW_INTERMEDIATE
    cv::imshow("AfterDetection", img);
    cv::waitKey(0);
  #endif

  std::cout << "Finished detection" << std::endl;
}

void SimpleDetectionMethod::fromJSON(const json11::Json &json){
  json11::Json appr_json = JSONCompatible::element_of(json, "ContoursApproximationThreshold");
  if(appr_json.type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("reading approximation threshold", "not receiving a number");

  contours_approximation_threshold = appr_json.int_value();

  appr_json = JSONCompatible::element_of(json, "MinBoardArea");
  if(appr_json.type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("reading minimum board area", "not receiving a number");

  min_board_area = appr_json.int_value();
}

/**
 * Overridden method to write class members to a Json object
 *
 * @return Returns a Json object containing the member variables of this object
 */
json11::Json SimpleDetectionMethod::toJSON() const {
  return json11::Json();
}


//  ████████╗███████╗███████╗████████╗
//  ╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝
//     ██║   █████╗  ███████╗   ██║
//     ██║   ██╔══╝  ╚════██║   ██║
//     ██║   ███████╗███████║   ██║
//     ╚═╝   ╚══════╝╚══════╝   ╚═╝
//  ███╗   ███╗ █████╗ ██╗███╗   ██╗
//  ████╗ ████║██╔══██╗██║████╗  ██║
//  ██╔████╔██║███████║██║██╔██╗ ██║
//  ██║╚██╔╝██║██╔══██║██║██║╚██╗██║
//  ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║
//  ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝
//
/*
int main(){
  std::string err;
  json11::Json json = json11::Json::parse("{\"Processing\":[\"Erosion\", [\"Rectangle\",222,222], \"Dilation\", [\"ellipse\", 100, 100], \"erosion\", [\"cross\", 150, 100]], \"Filter\":[0,0,0,180,255,255]}", err);

  cv::Mat img;
  SimpleDetectionMethod sdm(json);
  sdm.detect(img);
  return 0;
}
*/
