#include "MultiMorphology.hpp"
#include<iostream>//DEBUG

MultiMorphology::MultiMorphology(){
}

MultiMorphology::MultiMorphology(const json11::Json &json){
  fromJSON(json);
}

MultiMorphology::~MultiMorphology(){
  for(int i = 0; i < operations.size(); i++){
    delete operations[i];
  }
}

void MultiMorphology::fromJSON(const json11::Json &json){
  if(json.type() != json11::Json::ARRAY)
    throw JSONCompatible::InvalidJSON("retrieving MultiMorphology", "no array as input");

  auto arr = json.array_items();
  if(arr.size() % 2 != 0)
    throw JSONCompatible::InvalidJSON("retrieving MultiMorphology", "invalid number of elements in array");

  for(int i = 0; i < arr.size(); i += 2){
    operations.push_back(MorphologicalOperation::allocateFromJSON(json, i));
  }
}

json11::Json MultiMorphology::toJSON() const{ //TODO To implement
  return json11::Json();
}

void MultiMorphology::apply(cv::Mat &img) const{
  for(int i = 0; i < operations.size(); i++){
    operations[i]->apply(img);
  }
}

void MultiMorphology::applyCopy(const cv::Mat &src, cv::Mat &dst) const{
  if(operations.size() >= 1){
    operations[0]->applyCopy(src, dst);
    for(int i = 1; i < operations.size(); i++){
      operations[i]->apply(dst);
    }
  }
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
  json11::Json json = json11::Json::parse("{\"Processing\":[\"Erosion\", [\"Rectangle\",222,222], \"Dilation\", [\"ellipse\", 100, 100], \"erosion\", [\"cross\", 150, 100]]}", err);

  MultiMorphology mm(json["Processing"]);
  cv::Mat m;
  mm.apply(m);

  return 0;
}
*/
