#include "Filter.hpp"
#include<iostream>//DEBUG
#define ELEMENTS_IN_SCALAR 3
#define ELEMENTS_IN_FILTER 2*ELEMENTS_IN_SCALAR


//  ███████╗██╗██╗  ████████╗███████╗██████╗
//  ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗
//  █████╗  ██║██║     ██║   █████╗  ██████╔╝
//  ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗
//  ██║     ██║███████╗██║   ███████╗██║  ██║
//  ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝
//

Filter::Filter(cv::Scalar ls, cv::Scalar hs) : lows(ls), highs(hs){
}

Filter::Filter(unsigned char l1, unsigned char l2, unsigned char l3, unsigned char h1, unsigned char h2, unsigned char h3): lows(l1, l2, l3), highs(h1, h2, h3){
}

Filter::Filter(const json11::Json &json){
  fromJSON(json);
}

void Filter::fromJSON(const json11::Json &json){
  if(json.type() != json11::Json::ARRAY)
    throw JSONCompatible::InvalidJSON("retrieving Filter", "expecting but not receiveng an array");

  auto arr = json.array_items();
  if(arr.size() != ELEMENTS_IN_FILTER)
    throw JSONCompatible::InvalidJSON("retrieving Filter", "invalid number of elements in array");

  for(int i = 0; i < ELEMENTS_IN_FILTER; i++){
    if(arr[i].type() != json11::Json::NUMBER)
      throw JSONCompatible::InvalidJSON("retrieving Filter", "invalid element in array: expecting number");
  }
  //TODO: Check correctness of values
  lows[0] = arr[0].int_value();
  lows[1] = arr[1].int_value();
  lows[2] = arr[2].int_value();
  highs[0] = arr[3].int_value();
  highs[1] = arr[4].int_value();
  highs[2] = arr[5].int_value();
}

json11::Json Filter::toJSON() const{
  return json11::Json();
}

//  ██╗███╗   ██╗    ██████╗  █████╗ ███╗   ██╗ ██████╗ ███████╗
//  ██║████╗  ██║    ██╔══██╗██╔══██╗████╗  ██║██╔════╝ ██╔════╝
//  ██║██╔██╗ ██║    ██████╔╝███████║██╔██╗ ██║██║  ███╗█████╗
//  ██║██║╚██╗██║    ██╔══██╗██╔══██║██║╚██╗██║██║   ██║██╔══╝
//  ██║██║ ╚████║    ██║  ██║██║  ██║██║ ╚████║╚██████╔╝███████╗
//  ╚═╝╚═╝  ╚═══╝    ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝ ╚══════╝
//  ███████╗██╗██╗  ████████╗███████╗██████╗
//  ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗
//  █████╗  ██║██║     ██║   █████╗  ██████╔╝
//  ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗
//  ██║     ██║███████╗██║   ███████╗██║  ██║
//  ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝
//

void InRangeFilter::apply(cv::Mat &img) const{
  cv::inRange(img, lows, highs, img);
  //std::cout << "Applying Filter (" << lows[0] << "," << lows[1] << "," << lows[2] << ")->("  << highs[0] << ","  << highs[1] << ","  << highs[2] << ")" << std::endl;
}

void InRangeFilter::applyCopy(const cv::Mat &src, cv::Mat &dst) const{
  cv::inRange(src, lows, highs, dst);
  //std::cout << "Applying CopyFilter (" << lows[0] << "," << lows[1] << "," << lows[2] << ")->("  << highs[0] << ","  << highs[1] << ","  << highs[2] << ")" << std::endl;
}

//   █████╗ ██████╗  █████╗ ██████╗ ████████╗██╗██╗   ██╗███████╗
//  ██╔══██╗██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝██║██║   ██║██╔════╝
//  ███████║██║  ██║███████║██████╔╝   ██║   ██║██║   ██║█████╗
//  ██╔══██║██║  ██║██╔══██║██╔═══╝    ██║   ██║╚██╗ ██╔╝██╔══╝
//  ██║  ██║██████╔╝██║  ██║██║        ██║   ██║ ╚████╔╝ ███████╗
//  ╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝        ╚═╝   ╚═╝  ╚═══╝  ╚══════╝
//  ███████╗██╗██╗  ████████╗███████╗██████╗
//  ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗
//  █████╗  ██║██║     ██║   █████╗  ██████╔╝
//  ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗
//  ██║     ██║███████╗██║   ███████╗██║  ██║
//  ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝
//

//TODO:
void AdaptiveFilter::apply(cv::Mat &img) const{
  //cv::inRange(img, lows, highs, img);
  std::cout << "Applying AdaptiveFilter (" << lows[0] << "," << lows[1] << "," << lows[2] << ")->("  << highs[0] << ","  << highs[1] << ","  << highs[2] << ")" << std::endl;
}

//TODO:
void AdaptiveFilter::applyCopy(const cv::Mat &src, cv::Mat &dst) const{
  //cv::inRange(src, lows, highs, dst);
  std::cout << "Applying CopyAdaptiveFilter (" << lows[0] << "," << lows[1] << "," << lows[2] << ")->("  << highs[0] << ","  << highs[1] << ","  << highs[2] << ")" << std::endl;
}

//  ████████╗██╗    ██╗ ██████╗     ██████╗  █████╗ ███╗   ██╗ ██████╗ ███████╗███████╗
//  ╚══██╔══╝██║    ██║██╔═══██╗    ██╔══██╗██╔══██╗████╗  ██║██╔════╝ ██╔════╝██╔════╝
//     ██║   ██║ █╗ ██║██║   ██║    ██████╔╝███████║██╔██╗ ██║██║  ███╗█████╗  ███████╗
//     ██║   ██║███╗██║██║   ██║    ██╔══██╗██╔══██║██║╚██╗██║██║   ██║██╔══╝  ╚════██║
//     ██║   ╚███╔███╔╝╚██████╔╝    ██║  ██║██║  ██║██║ ╚████║╚██████╔╝███████╗███████║
//     ╚═╝    ╚══╝╚══╝  ╚═════╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝ ╚══════╝╚══════╝
//  ███████╗██╗██╗  ████████╗███████╗██████╗
//  ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗
//  █████╗  ██║██║     ██║   █████╗  ██████╔╝
//  ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗
//  ██║     ██║███████╗██║   ███████╗██║  ██║
//  ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝
//
TwoRangesFilter::TwoRangesFilter(cv::Scalar ls1, cv::Scalar hs1, cv::Scalar ls2, cv::Scalar hs2): f1(ls1, hs1), f2(ls2, hs2){
}

TwoRangesFilter::TwoRangesFilter(const json11::Json &json) : f1(JSONCompatible::element_of(json, "Range1")), f2(JSONCompatible::element_of(json, "Range2")){
}

void TwoRangesFilter::fromJSON(const json11::Json &json){
  f1.fromJSON(JSONCompatible::element_of(json, "Range1"));
  f2.fromJSON(JSONCompatible::element_of(json, "Range2"));
}

json11::Json TwoRangesFilter::toJSON() const{
  return json11::Json();
}

//TODO:
void TwoRangesFilter::apply(cv::Mat &img) const{
  cv::Mat tmp;
  f1.applyCopy(img, tmp);
  f2.apply(img);
  cv::addWeighted(img, 1.0, tmp, 1.0, 0.0, img); // combine together the two binary masks
}

//TODO:
void TwoRangesFilter::applyCopy(const cv::Mat &src, cv::Mat &dst) const{
  cv::Mat tmp;
  f1.applyCopy(src, tmp);
  f2.applyCopy(src, dst);
  cv::addWeighted(dst, 1.0, tmp, 1.0, 0.0, dst); // combine together the two binary masks
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
  json11::Json json = json11::Json::parse("{\"Filter\":{\"Range1\":[0,0,0,898,255,255], \"Range2\":[0,0,0,832,432,432]}}", err);

  InRangeFilter f(json["Filter"]["Range1"]);
  cv::Mat m;
  f.apply(m);

  return 0;
}
*/
