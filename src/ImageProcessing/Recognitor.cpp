#include"Recognitor.hpp"
using namespace LaR;
#define SHOW_INTERMEDIATE

cv::Mat getPaddedROI(const cv::Mat &input, int top_left_x, int top_left_y, int width, int height, cv::Scalar paddingColor) {
  int bottom_right_x = top_left_x + width;
  int bottom_right_y = top_left_y + height;

  cv::Mat output;
  if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows) {
      // border padding will be required
      int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

      if (top_left_x < 0) {
          width = width + top_left_x;
          border_left = -1 * top_left_x;
          top_left_x = 0;
      }
      if (top_left_y < 0) {
          height = height + top_left_y;
          border_top = -1 * top_left_y;
          top_left_y = 0;
      }
      if (bottom_right_x > input.cols) {
          width = width - (bottom_right_x - input.cols);
          border_right = bottom_right_x - input.cols;
      }
      if (bottom_right_y > input.rows) {
          height = height - (bottom_right_y - input.rows);
          border_bottom = bottom_right_y - input.rows;
      }

      cv::Rect R(top_left_x, top_left_y, width, height);
      cv::copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, cv::BORDER_CONSTANT, paddingColor);
  }
  else {
      // no border padding required
      cv::Rect R(top_left_x, top_left_y, width, height);
      output = input(R);
  }
  return output;
}


Recognitor::Recognitor(const json11::Json &json):
            blue_filter(JSONCompatible::element_of(json, "BlueFilter")),
            green_filter(JSONCompatible::element_of(json, "GreenFilter")),
            black_green_filter(JSONCompatible::element_of(json, "BlackGreenFilter")),
            red_filter(JSONCompatible::element_of(json, "RedFilter")),
            blue_processing(JSONCompatible::element_of(json, "BlueProcessing")),
            green_processing(JSONCompatible::element_of(json, "GreenProcessing")),
            black_green_processing(JSONCompatible::element_of(json, "BlackGreenProcessing")),
            red_processing(JSONCompatible::element_of(json, "RedProcessing")),
            gate_contour_finder(JSONCompatible::element_of(json, "GateContourApproximation")),
            obstacles_contour_finder(JSONCompatible::element_of(json, "ObstaclesContourApproximation")),
            victims_contour_finder(JSONCompatible::element_of(json, "VictimsContourApproximation")){

  json11::Json appr_json = JSONCompatible::element_of(json, "MinimumDigitArea");
  if(appr_json.type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("reading minimum digit area", "not receiving a number");

  min_digit_area = appr_json.int_value();

  appr_json = JSONCompatible::element_of(json, "DigitImageSize");
  if(appr_json.type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("reading digit image size", "not receiving a number");

  digit_img_size = appr_json.int_value();
}

void Recognitor::recognize(const cv::Mat &img, Map &m){
  Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

  std::vector<Obstacle> obstacles;
  Obstacle gate;
  std::vector<Victim> victims;
  getObstacles(hsv_img, obstacles);
  getGate(hsv_img, gate);
  getCircles(img, hsv_img, victims);

  m = Map(1470, 970, gate, obstacles, victims);
}

void Recognitor::getObstacles(const cv::Mat &img, std::vector<Obstacle> &obstacles) const{
  obstacles.clear();

  cv::Mat red_mask;
  red_filter.applyCopy(img, red_mask);
  red_processing.apply(red_mask);

  #ifdef SHOW_INTERMEDIATE_RECOGNITION
    imshow("OBSTACLES", red_mask);
    waitKey(0);
  #endif

  obstacles_contour_finder.findContours(red_mask, obstacles);

  #ifdef PRINT_INTERMEDIATE
    std::cout << "Total number of obstacles found: " << obstacles.size() << std::endl;
  #endif

  #if defined SHOW_INTERMEDIATE_RECOGNITION || defined SHOW_INTERMEDIATE
    cv::Mat tmp = img.clone();
    std::vector<std::vector<cv::Point>> contours(obstacles.size());
    for(int i = 0; i < obstacles.size(); i++){
      for(int k = 0; k < obstacles[i].getNVertices(); k++){
        contours[i].push_back(cv::Point(obstacles[i].getVertices()[k].x, obstacles[i].getVertices()[k].y));
      }
    }
    cv::drawContours(tmp, contours, -1, Scalar(255,0,0), 3, cv::LINE_AA);

    imshow("OBSTACLES", tmp);
    waitKey(0);
  #endif
}

void Recognitor::getGate(const cv::Mat &img, Obstacle &gate) const{
  gate = Obstacle();
  // Find blue regions
  Mat blue_mask;
  blue_filter.applyCopy(img, blue_mask);
  blue_processing.apply(blue_mask);
  #ifdef SHOW_INTERMEDIATE_RECOGNITION
    imshow("GATE", blue_mask);
    waitKey(0);
  #endif

  gate_contour_finder.findContours(blue_mask, gate);

  #ifdef PRINT_INTERMEDIATE
    std::cout << "Gate found: " <<;
    gate.print();
  #endif

  #if defined SHOW_INTERMEDIATE_RECOGNITION || defined SHOW_INTERMEDIATE
    cv::Mat tmp = img.clone();
    std::vector<std::vector<cv::Point>> contours(1);
    for(int k = 0; k < gate.getNVertices(); k++){
      contours[0].push_back(cv::Point(gate.getVertices()[k].x, gate.getVertices()[k].y));
    }
    cv::drawContours(tmp, contours, -1, Scalar(255,0,0), 3, cv::LINE_AA);

    imshow("GATE", tmp);
    waitKey(0);
  #endif
}

void Recognitor::getCircles(const cv::Mat &original, const cv::Mat &hsv, std::vector<Victim> &victims) const{
  victims.clear();

  Mat black_green_mask;
  black_green_filter.applyCopy(hsv, black_green_mask);
  black_green_processing.apply(black_green_mask);

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);

  cv::GaussianBlur(channels[0], channels[0], cv::Size(9, 9), 9);
  cv::adaptiveThreshold(channels[0], channels[0], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 405, -7);
  cv::bitwise_and(channels[0], black_green_mask, black_green_mask);

  #ifdef SHOW_INTERMEDIATE_RECOGNITION
    imshow("VICTIMS", black_green_mask);
    waitKey(0);
  #endif

  victims_contour_finder.findContours(black_green_mask, victims);

  #if defined SHOW_INTERMEDIATE_RECOGNITION || defined SHOW_INTERMEDIATE
    std::cout << "VICTIMS FOUND: " << victims.size() << std::endl;
    cv::Mat contour_mask = original.clone();
    for(int crc = 0; crc < victims.size(); crc++){
      std::cout<< "Victim [" << crc << "]: (" << victims[crc].getCenter().x << ", " << victims[crc].getCenter().y << ") -> R: " << victims[crc].getRadius() << std::endl;
      cv::circle(contour_mask, cv::Point2f(victims[crc].getCenter().x, victims[crc].getCenter().y), victims[crc].getRadius(), Scalar(255,0,0), 3, cv::LINE_AA);
    }
    cv::imshow("VICTIMS", contour_mask);
    cv::waitKey(0);
  #endif

  // Load digits template images
  std::vector<cv::Mat> templROIs;
  for (int i = 0; i < 20; i++) {
    templROIs.push_back(cv::Mat(cv::imread("./Template_images/M_" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE)));
  }


  for (int i=0; i<victims.size(); i++) {
    #ifdef PRINT_INTERMEDIATE
      std::cout << "PROCESSING VICTIM " << i << std::endl;
    #endif

    cv::Rect b_rect(victims[i].getCenter().x - victims[i].getRadius(), victims[i].getCenter().y - victims[i].getRadius(), victims[i].getRadius() * 2, victims[i].getRadius() * 2);
    cv::Mat original_subsection(original, b_rect);
    cv::Mat processROI = original_subsection.clone();
    cv::Mat localFilter(black_green_mask, b_rect);

    #ifdef SHOW_INTERMEDIATE_RECOGNITION
      cv::imshow("ProcessROI", processROI);
      cv::waitKey(0);
    #endif

    cv::cvtColor(processROI, processROI, cv::COLOR_BGR2YUV);

    std::vector<cv::Mat> channels;
    cv::split(processROI, channels);
    cv::threshold(channels[2], processROI, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    cv::erode(localFilter, localFilter, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::bitwise_and(processROI, localFilter, processROI);

    cv::dilate(processROI, processROI, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    //cv::erode(processROI, processROI, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    #ifdef SHOW_INTERMEDIATE_RECOGNITION
      cv::imshow("ProcessROI", processROI);
      cv::waitKey(0);
    #endif

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(processROI, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    #ifdef SHOW_INTERMEDIATE_RECOGNITION
      cv::Mat original_clone = original.clone();
      cv::Mat contour_mask(original_clone, b_rect);
      cv::drawContours(contour_mask, contours, -1, Scalar(255,0,0), 3, cv::LINE_AA);
      cv::imshow("DIGIT CONTOURS", contour_mask);
      cv::waitKey(0);
    #endif

    int recognized_number = -1;
    std::vector<cv::Point> approx_curve;

    float max_area = 0;
    int best_index = -1;
    for (int j=0; j < contours.size(); ++j)   {
      #ifdef PRINT_INTERMEDIATE
        std::cout << "Contours found: " << cv::contourArea(contours[j]) << std::endl;
      #endif

      float area = cv::contourArea(contours[j]);
      if(area > max_area){
        best_index = j;
        max_area = area;
      }
    }

    //TODO: Define a minimum max area in JSON
    if(best_index == -1 || max_area < 2000)
      throw std::logic_error("Unable to find a big enough contour for one of the digits");

    cv::approxPolyDP(contours[best_index], approx_curve, 1, true);


    cv::Vec4f line;
    cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
    double vx = line[0], vy = line[1], x = line[2], y = line[3];
    double lefty = cvRound((-x * vy / vx) + y);
    double righty = cvRound(((processROI.cols - x) * vy / vx) + y);
    cv::Point point1 = cv::Point(processROI.cols - 1, righty);
    cv::Point point2 = cv::Point(0, lefty);

    #ifdef SHOW_INTERMEDIATE_RECOGNITION
      cv::Mat linedROI = processROI.clone();
      cv::line(linedROI, point1, point2, cv::Scalar(255,0,0), 2, cv::LINE_AA, 0);
      cv::imshow("ProcessROI", linedROI);
      cv::waitKey(0);
    #endif

    float rotAng = cv::fastAtan2(vy, vx) + 90;
    cv::RotatedRect numberBounds = cv::minAreaRect(cv::Mat(approx_curve));
    cv::Point2f center = numberBounds.center;

    cv::Mat r = cv::getRotationMatrix2D(center, rotAng, 1.0);

    cv::warpAffine(processROI, processROI, r, cv::Size(200, 200));
    cv::bitwise_not(processROI, processROI);

    cv::Mat centeredImage = getPaddedROI(processROI, center.x - 100, center.y - 100, 200, 200, cv::Scalar(255,255,255));

    #if defined SHOW_INTERMEDIATE_RECOGNITION || defined SHOW_INTERMEDIATE
      cv::imshow("VICTIMS", centeredImage);
      cv::waitKey(0);
    #endif

    // Find the template digit with the best matching
    double maxScore = 0;
    int maxIdx = -1;
    for (int templ_i = 0; templ_i < templROIs.size(); templ_i++) {
      cv::Mat result;
      cv::matchTemplate(centeredImage, templROIs[templ_i], result, cv::TM_CCOEFF);
      double score;
      cv::minMaxLoc(result, nullptr, &score);

      if (score > maxScore) {
        maxScore = score;
        maxIdx = templ_i;
      }
    }

    recognized_number = maxIdx/2;
    std::cout << "Number recognized as " << recognized_number << std::endl;
    victims[i].setNumber(recognized_number);
  }
}

void Recognitor::fromJSON(const json11::Json &json){
  json.type();
}

json11::Json Recognitor::toJSON() const {
  return json11::Json();
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

  Recognitor r(JSONCompatible::element_of(json, "RecognitionParameters"));
  r.recognize(img);
  cv::imshow("BLOH", img);
  cv::waitKey(0);

  return 0;
}*/
