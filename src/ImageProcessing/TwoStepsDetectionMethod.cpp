#include"TwoStepsDetectionMethod.hpp"
#include <vector>
#include <algorithm>
#define SHOW_INTERMEDIATE
#define SHOW_INTERMEDIATE_DETECTION

TwoStepsDetectionMethod::TwoStepsDetectionMethod(const json11::Json json) :
                        black_filter_1(JSONCompatible::element_of(json, "Filter1")),
                        black_filter_2(JSONCompatible::element_of(json, "Filter2")),
                        processing_1(JSONCompatible::element_of(json, "Processing1")),
                        processing_2(JSONCompatible::element_of(json, "Processing2")){
    json11::Json appr_json = JSONCompatible::element_of(json, "ContoursApproximationThreshold1");
    if(appr_json.type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("reading approximation threshold 1", "not receiving a number");

    contours_approximation_threshold_1 = appr_json.int_value();

    appr_json = JSONCompatible::element_of(json, "ContoursApproximationThreshold2");
    if(appr_json.type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("reading approximation threshold 2", "not receiving a number");

    contours_approximation_threshold_2 = appr_json.int_value();

    appr_json = JSONCompatible::element_of(json, "BoardFirstOffset");
    if(appr_json.type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("reading board offset", "not receiving a number");

    board_first_offset = appr_json.int_value();

    appr_json = JSONCompatible::element_of(json, "BoardFirstExpansion");
    if(appr_json.type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("reading board expansion", "not receiving a number");

    board_first_expansion = appr_json.int_value();

    appr_json = JSONCompatible::element_of(json, "MinAreaSecondPerspective");
    if(appr_json.type() != json11::Json::NUMBER)
        throw JSONCompatible::InvalidJSON("reading board min area", "not receiving a number");

    min_area_second_perspective = appr_json.int_value();
}

void TwoStepsDetectionMethod::detect(cv::Mat & img, int board_width, int board_height) const{
  cv::Mat tmp;
  std::vector<cv::Point> src_pts;
  std::vector<cv::Point2f> dst_pts;

  black_filter_1.applyCopy(img, tmp);
  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("AfterFilter1", tmp);
    cv::waitKey(0);
  #endif

  processing_1.apply(tmp);

  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("AfterProcessing1", tmp);
    cv::waitKey(0);
  #endif

  findBoardContours(tmp, src_pts, false);

  #ifdef SHOW_INTERMEDIATE_DETECTION
  {
    cv::Mat tmpImg = img.clone();
    std::vector<std::vector<cv::Point>> contours_approx = {src_pts};
    cv::drawContours(tmpImg, contours_approx, -1, cv::Scalar(255, 65, 65), 2, cv::LINE_AA);
    cv::imshow("CONTOURS", tmpImg);
    cv::waitKey(0);
  }
  #endif
  cv::Size board_size(board_width, board_height);
  cv::Size first_size(board_size.width + board_first_expansion, board_size.height + board_first_expansion);
  const int x0 = board_first_offset, y0 = board_first_offset, x1 = first_size.width - board_first_offset, y1 = first_size.height - board_first_offset;
  dst_pts = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};

  // Find the homography between the board contour points and the destination points
  cv::Mat h = cv::findHomography(src_pts, dst_pts);

  // Warp the perspective and show the final image
  cv::warpPerspective(img, img, h, first_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("AfterWarping", img);
      cv::waitKey(0);
  #endif

  black_filter_2.applyCopy(img, tmp);

  #ifdef SHOW_INTERMEDIATE_DETECTION
    cv::imshow("AfterFilter2", tmp);
    cv::waitKey(0);
    #endif

  processing_2.apply(tmp);
  #ifdef SHOW_INTERMEDIATE_DETECTION
      cv::imshow("AfterProcessing2", tmp);
      cv::waitKey(0);
  #endif

  findBoardContours(tmp, src_pts, true);

  #ifdef SHOW_INTERMEDIATE_DETECTION
  {
    cv::Mat tmpImg = img.clone();
    std::vector<std::vector<cv::Point>> contours_approx = {src_pts};
    cv::drawContours(tmpImg, contours_approx, -1, cv::Scalar(255, 65, 65), 2, cv::LINE_AA);
    cv::imshow("CONTOURS2", tmpImg);
    cv::waitKey(0);
  }
  #endif


  dst_pts = {{0,0}, {board_size.width, 0}, {board_size.width, board_size.height}, {0, board_size.height}};
  h = cv::findHomography(src_pts, dst_pts);
  cv::warpPerspective(img, img, h, board_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

  #ifdef SHOW_INTERMEDIATE
    cv::imshow("AfterDetection", img);
    cv::waitKey(0);
  #endif
}

void TwoStepsDetectionMethod::fromJSON(const json11::Json &json){
  json11::Json j = json;
}

json11::Json TwoStepsDetectionMethod::toJSON() const{
  return json11::Json();
}

void TwoStepsDetectionMethod::findBoardContours(const cv::Mat &mask, std::vector<cv::Point> &src_pts, bool second) const{
    // Variables for the contours detection
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> &approx_curve = src_pts;

    // Find the external contours of the black mask
    cv::findContours(mask, contours, (second ? cv::RETR_LIST : cv::RETR_EXTERNAL), cv::CHAIN_APPROX_SIMPLE);

    // Variables needed to find the biggest rectangular contour by area
    double max_area = 0, second_max_area = 0;
    int biggest_area_position = 0, second_biggest_area_position = 0;

    // Iterate each contour and find the biggest rectangular contour that is the board
    for (int i=0; i<contours.size(); ++i)   {
        // fit a closed polygon (with less vertices) to the given contour, with an approximation accuracy of black_contour_approx
        cv::approxPolyDP(contours[i], approx_curve, (second ? contours_approximation_threshold_2 : contours_approximation_threshold_1), true);

        // Calculate the area of the contour
        double area = cv::contourArea(contours[i]);

        // If this contour is reactangular and have the biggest rectangular area so far
        if(approx_curve.size() == BOARD_N_VERTICES){
            if(area > max_area) { //TODO: CONSTANT
                // Update the biggest area and the position of this contour
                if(!second){
                    max_area = area;
                    biggest_area_position = i;
                } else if(area > min_area_second_perspective){
                    second_max_area = max_area;
                    max_area = area;
                    second_biggest_area_position = biggest_area_position;
                    biggest_area_position = i;
                }

                #ifdef PRINT_INTERMEDIATE
                    std::cout << "Approximated contour size of contour " << i << " : " << approx_curve.size() << std::endl;
                    std::cout << "AREA: " << area << std::endl;
                #endif
            } else if(second && area > min_area_second_perspective && area > second_max_area){
                std::cout << " ASSIGNING SECOND SHIT " << std::endl;
                second_max_area = area;
                second_biggest_area_position = i;
            }
        }
    }


    // Draw and show the biggest rectangular contour found
    std::vector<cv::Point> board_contour_raw = contours[(second ? second_biggest_area_position : biggest_area_position)];
    cv::approxPolyDP(board_contour_raw, approx_curve, (second ? contours_approximation_threshold_2 : contours_approximation_threshold_1), true); //TODO: AVOID RECOMPUTING APPROX


    #ifdef PRINT_INTERMEDIATE
        // Print the coordinates of the board in the image
        for(int i = 0; i < approx_curve.size(); i++) {
          std::cout << approx_curve[i] << " ";
        }
        std::cout << std::endl;
    #endif

    cv::Point2f center((approx_curve[0].x + approx_curve[1].x + approx_curve[2].x + approx_curve[3].x)/4, (approx_curve[0].y + approx_curve[1].y + approx_curve[2].y + approx_curve[3].y)/4);
    class angle_index{public: float angle; char index; cv::Point point; angle_index(float a, char i, cv::Point& p):angle(a), index(i), point(p){} bool operator <(const angle_index& other) const{return angle < other.angle;}};
    std::vector<angle_index> angles;
    for(int ind = 0; ind < 4; ind++) angles.push_back(angle_index(atan2(approx_curve[ind].y - center.y, approx_curve[ind].x - center.x), ind, approx_curve[ind]));
    std::sort(angles.begin( ), angles.end( ));

    // Calculate the distance between each adjacent board point
    float distance_0_to_1 = sqrt( pow(angles[0].point.x - angles[1].point.x,2) +  pow(angles[0].point.y - angles[1].point.y,2) );
    float distance_3_to_0 = sqrt( pow(angles[3].point.x - angles[0].point.x,2) +  pow(angles[3].point.y - angles[0].point.y,2) );

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

/*
#include"Recognitor.hpp"
int main(){
  std::string err;
  std::ifstream f("input/file.json");
  std::stringstream buf;
  buf << f.rdbuf();
  std::string json_string = buf.str();
  json11::Json json = json11::Json::parse(json_string, err);

  cv::Mat img = cv::imread("test_image.jpg");

  TwoStepsDetectionMethod sdm(JSONCompatible::element_of(JSONCompatible::element_of(json,"DetectionParameters"),"TwoStepsDetectionMethod"));
  sdm.detect(img);
  cv::imshow("BLOH", img);
  cv::waitKey(0);

  return 0;
}
*/
