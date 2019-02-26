#include "Detector.hpp"
#include <chrono>
#include <exception>
//#define SHOW_INTERMEDIATE_DETECTION

Detector::Detector(json11::Json json) :
          robot_filter(JSONCompatible::element_of(JSONCompatible::read_json(json, "input/detection_parameters.json"), "LightBlueFilter")),
          robot_processing(JSONCompatible::element_of(json, "LightBlueProcessing")),
          robot_contour_finder(JSONCompatible::element_of(json, "LightBlueContourApproximation")),
          corners_filter(JSONCompatible::element_of(json, "WhiteFilter")),
          corners_processing(JSONCompatible::element_of(json, "WhiteProcessing")),
          corners_contour_finder(JSONCompatible::element_of(json, "WhiteContourFinder")),
          detection_method(JSONCompatible::element_of(json, "SimpleDetectionMethod")){
  fromJSON(json);
}

bool Detector::localize(const cv::Mat &img, std::vector<double> & state){
  auto start = std::chrono::high_resolution_clock::now();

  cv::Mat tmp;

  cv::Size board_size(board_width + tape_width/2.0, board_height + tape_width/2.0);
  cv::warpPerspective(img, tmp, robot_plane_homography, board_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
  //#define SHOW_INTERMEDIATE_DETECTION
  #if defined SHOW_INTERMEDIATE_DETECTION || defined SHOW_INTERMEDIATE
    cv::Mat tmpImg = tmp.clone();
  #endif

  cv::cvtColor(tmp, tmp, CV_BGR2HSV);

  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("LOCALIZING", tmp);
    cv::waitKey(0);
  #endif

  robot_filter.apply(tmp);
  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("LOCALIZING", tmp);
    cv::waitKey(0);
  #endif

  robot_processing.apply(tmp);
  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("LOCALIZING", tmp);
    cv::waitKey(0);
  #endif

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Point> approx_curve, biggest_triangle;
  cv::Point2f best_center;
  double max_area = 0;

  cv::findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (int i = 0; i < contours.size(); ++i)  {
    cv::approxPolyDP(contours[i], approx_curve, 30, true);

    if(approx_curve.size() == 3 && cv::contourArea(approx_curve) > max_area){
      max_area = cv::contourArea(approx_curve);
      biggest_triangle = approx_curve;
      cv::Moments mu = moments(approx_curve, false);
      best_center = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);

      #ifdef SHOW_INTERMEDIATE_DETECTION
      {
        std::vector<std::vector<cv::Point>> contours_approx = {approx_curve};
        cv::drawContours(tmpImg, contours_approx, -1, cv::Scalar(255, 65, 65), 2, cv::LINE_AA);
        cv::circle(tmpImg, cv::Point(best_center.x, best_center.y), 3, cv::Scalar(255,0,0), cv::FILLED);
      }
      #endif
    }
  }
  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("LOCALIZING", tmpImg);
    cv::waitKey(0);
  #endif

  if(biggest_triangle.size() != 3)
	return false;
    //throw std::logic_error("Unable to find robot");

  cv::Point d = biggest_triangle[1] - biggest_triangle[0];
  double min_len = std::sqrt(d.x*d.x + d.y*d.y);
  int min_ind = 0;
  for(int i = 1; i < 3; i++){
    cv::Point dt = biggest_triangle[(i+1)%3] - biggest_triangle[i];
    double l_tmp = std::sqrt(dt.x*dt.x + dt.y*dt.y);
    if(l_tmp < min_len){
      min_len = l_tmp;
      min_ind = i;
    }
  }

  cv::Point base_middle = (biggest_triangle[(min_ind+1)%3] + biggest_triangle[min_ind])/2;
  state.clear();
  state.push_back(best_center.x);
  state.push_back(best_center.y);
  state.push_back(std::atan2(base_middle.y - best_center.y, base_middle.x - best_center.x));

  #if defined SHOW_INTERMEDIATE_DETECTION || defined SHOW_INTERMEDIATE
    std::cout << "THETA: " << state[2] << std::endl;
    cv::arrowedLine(tmpImg, best_center, base_middle, cv::Scalar(255,0,0), 2);
    cv::imshow("LOCALIZING", tmpImg);
    cv::waitKey(0);
  #endif
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> localisation_time = finish - start;
  std::cout << "LOC: " << localisation_time.count() <<std::endl;
	return true;
}

void Detector::detect_board_and_warp(cv::Mat &img){
  detection_method.detect(img, board_width, board_height);
}

void Detector::detect_robot_plane(const cv::Mat &img){
  /*
  cv::Mat tmp;
  cv::cvtColor(img, tmp, CV_BGR2HSV);
  #define SHOW_INTERMEDIATE_DETECTION

  corners_filter.apply(tmp);
  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("AfterCornerFiltering", tmp);
    cv::waitKey(0);
  #endif

  corners_processing.apply(tmp);

  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("AfterCornerProcessing", tmp);
    cv::waitKey(0);
  #endif


*/
  std::vector<cv::Point> src_pts;
  corners_contour_finder.findContours(img, src_pts);

  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::Mat circlesIMG = img.clone();
    std::cout << "CIRCLES: " << src_pts.size() << std::endl;
    for(int k = 0; k < src_pts.size(); k++){
      std::cout << src_pts[k].x << " " << src_pts[k].y << " " << std::endl;
      cv::circle(circlesIMG, src_pts[k], 5, cv::Scalar(255,0,0), cv::FILLED);
    }
    cv::imshow("HOUGH CIRCLES", circlesIMG);
    cv::waitKey(0);
  #endif

/*

  for (int i = 0; i < contours.size(); ++i)  {
    cv::approxPolyDP(contours[i], approx_curve, 10, true);

    if(cv::contourArea(approx_curve) > 100){
      cv::Moments mu = moments(approx_curve, false);
      cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);
      src_pts.push_back(cv::Point(center.x, center.y));
      #ifdef SHOW_INTERMEDIATE_DETECTION
      {
        std::vector<std::vector<cv::Point>> contours_approx = {approx_curve};
        //cv::drawContours(tmpImg, contours_approx, -1, cv::Scalar(255, 65, 65), 2, cv::LINE_AA);
        cv::circle(tmpImg, cv::Point(center.x, center.y), 3, cv::Scalar(255,0,0), cv::FILLED);
      }
      #endif
    }
  }
  */

  cv::Point2f center((src_pts[0].x + src_pts[1].x + src_pts[2].x + src_pts[3].x)/4, (src_pts[0].y + src_pts[1].y + src_pts[2].y + src_pts[3].y)/4);
  class angle_index{
    public: float angle; char index; cv::Point point;
    angle_index(float a, char i, cv::Point& p):angle(a), index(i), point(p){}
    bool operator <(const angle_index& other) const{return angle < other.angle;}};
  std::vector<angle_index> angles;
  for(int ind = 0; ind < 4; ind++) angles.push_back(angle_index(atan2(src_pts[ind].y - center.y, src_pts[ind].x - center.x), ind, src_pts[ind]));
  std::sort(angles.begin( ), angles.end( ));

  // Calculate the distance between each adjacent board point
  float distance_0_to_1 = std::sqrt( std::pow(angles[0].point.x - angles[1].point.x,2) +  std::pow(angles[0].point.y - angles[1].point.y,2) );
  float distance_3_to_0 = std::sqrt( std::pow(angles[3].point.x - angles[0].point.x,2) +  std::pow(angles[3].point.y - angles[0].point.y,2) );

  if(distance_0_to_1 > distance_3_to_0) {
      src_pts[0] = angles[0].point;
      src_pts[1] = angles[1].point;
      src_pts[2] = angles[2].point;
      src_pts[3] = angles[3].point;
  } else {
      src_pts[0] = angles[1].point;
      src_pts[1] = angles[2].point;
      src_pts[2] = angles[3].point;
      src_pts[3] = angles[0].point;
  }




  cv::Size board_size(board_width + tape_width/2.0, board_height + tape_width/2.0);
  std::vector<cv::Point> dst_pts = {{0,0}, {board_size.width, 0}, {board_size.width, board_size.height}, {0, board_size.height}};
  robot_plane_homography = cv::findHomography(src_pts, dst_pts);

  #if defined SHOW_INTERMEDIATE_DETECTION || defined SHOW_INTERMEDIATE
    cv::Mat imgClone = img.clone();
    cv::warpPerspective(imgClone, imgClone, robot_plane_homography, board_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
    cv::imshow("WARPED", imgClone);
    cv::waitKey(0);
  #endif
}

void Detector::fromJSON(const json11::Json &json){
  json11::Json board_json = JSONCompatible::element_of(json, "BoardSize");

  if(board_json.type() != json11::Json::ARRAY)
    throw JSONCompatible::InvalidJSON("reading board size", "not receiving array");

  auto arr = board_json.array_items();
  if(arr.size() > 3)
    throw JSONCompatible::InvalidJSON("reading board size", "wrong array dimension");

  if(arr[0].type() != json11::Json::NUMBER || arr[1].type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading board size", "not receiving numbers");

  board_width = arr[0].number_value();
  board_height = arr[1].number_value();
  tape_width = arr[2].int_value();

  if(board_width <= 0 || board_height <= 0 || tape_width <= 0)
    throw JSONCompatible::InvalidJSON("retrieving board size", "negative value for width or height");
}

json11::Json Detector::toJSON() const{
  return json11::Json();
}
