#include"MorphologicalOperation.hpp"

//  ███╗   ███╗ ██████╗ ██████╗ ██████╗ ██╗  ██╗ ██████╗ ██╗      ██████╗  ██████╗ ██╗ ██████╗ █████╗ ██╗
//  ████╗ ████║██╔═══██╗██╔══██╗██╔══██╗██║  ██║██╔═══██╗██║     ██╔═══██╗██╔════╝ ██║██╔════╝██╔══██╗██║
//  ██╔████╔██║██║   ██║██████╔╝██████╔╝███████║██║   ██║██║     ██║   ██║██║  ███╗██║██║     ███████║██║
//  ██║╚██╔╝██║██║   ██║██╔══██╗██╔═══╝ ██╔══██║██║   ██║██║     ██║   ██║██║   ██║██║██║     ██╔══██║██║
//  ██║ ╚═╝ ██║╚██████╔╝██║  ██║██║     ██║  ██║╚██████╔╝███████╗╚██████╔╝╚██████╔╝██║╚██████╗██║  ██║███████╗
//  ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝
//   ██████╗ ██████╗ ███████╗██████╗  █████╗ ████████╗██╗ ██████╗ ███╗   ██╗
//  ██╔═══██╗██╔══██╗██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║
//  ██║   ██║██████╔╝█████╗  ██████╔╝███████║   ██║   ██║██║   ██║██╔██╗ ██║
//  ██║   ██║██╔═══╝ ██╔══╝  ██╔══██╗██╔══██║   ██║   ██║██║   ██║██║╚██╗██║
//  ╚██████╔╝██║     ███████╗██║  ██║██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║
//   ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
//
const std::unordered_map<std::string, int> MorphologicalOperation::encode_kernel{{"rectangle", cv::MORPH_RECT}, {"Rectangle", cv::MORPH_RECT}, {"ellipse", cv::MORPH_ELLIPSE}, {"Ellipse", cv::MORPH_ELLIPSE}, {"cross", cv::MORPH_CROSS}, {"Cross", cv::MORPH_CROSS}};
const std::unordered_map<std::string,std::function<MorphologicalOperation*(const json11::Json &json)>>
                         MorphologicalOperation::encode_operation_type{
                           {"Dilation", [](const json11::Json &json){return new Dilation(json);}},
                           {"dilation", [](const json11::Json &json){return new Dilation(json);}},
                           {"Erosion", [](const json11::Json &json){return new Erosion(json);}},
                           {"erosion", [](const json11::Json &json){return new Erosion(json);}},
                           {"Gaussian", [](const json11::Json &json){return new Gaussian(json);}},
                           {"gaussian", [](const json11::Json &json){return new Gaussian(json);}}
                         };

/* PUBLIC METHODS*/

MorphologicalOperation::MorphologicalOperation(int shape, cv::Size size) : kernel_shape(shape), kernel_size(size){
  if(shape != cv::MORPH_RECT || shape != cv::MORPH_CROSS || shape != cv::MORPH_ELLIPSE || size.height <= 0 || size.width <= 0){
    shape = -1;
    size = cv::Size(0,0);
  }
};

MorphologicalOperation::MorphologicalOperation(const json11::Json &json){
  fromJSON(json);
}

void MorphologicalOperation::fromJSON(const json11::Json &json){
  if(json.type() != json11::Json::ARRAY)
    throw JSONCompatible::InvalidJSON("reading kernel info", "not receiving an array");

  if(json.array_items().size() < 3)
    throw JSONCompatible::InvalidJSON("reading kernel info", "wrong number of parameters");

  readKernelShape(json[0]);
  readKernelSize(json[1], json[2]);
}

MorphologicalOperation* MorphologicalOperation::allocateFromJSON(const json11::Json &json, int i){
  auto it = encode_operation_type.find(json[i].string_value());
  if(it != encode_operation_type.end())
    return it->second(json[i+1]);
  else{
    throw JSONCompatible::InvalidJSON("reading morphological operation type", "unknown operation name");
  }
}

//TODO: To implement
json11::Json MorphologicalOperation::toJSON() const {
  return json11::Json();
}

/*PRIVATE METHODS*/
void MorphologicalOperation::readKernelShape(const json11::Json &json){
  if(json.type() == json11::Json::NUMBER){
    kernel_shape = json.int_value();
    if(kernel_shape != cv::MORPH_RECT && kernel_shape != cv::MORPH_ELLIPSE && kernel_shape != cv::MORPH_CROSS)
      throw JSONCompatible::InvalidJSON("reading kernel shape", "the undefined shape value: " + std::to_string(json.int_value()));
  } else if(json.type() == json11::Json::STRING){
    auto it = encode_kernel.find(json.string_value());
    if(it != encode_kernel.end())
      kernel_shape = it->second;
    else
      throw JSONCompatible::InvalidJSON("reading kernel shape", "the string value " + json.string_value());
  }
  else
    throw JSONCompatible::InvalidJSON("reading kernel shape",  "not acceptable shape format");
}

void MorphologicalOperation::readKernelSize(const json11::Json &json1, const json11::Json &json2){
  if(json1.type() != json11::Json::NUMBER || json2.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading kernel size", "not receiving 2 numbers");

  int j1 = json1.int_value(), j2 = json2.int_value();
  if( j1 <= 0 ||  j2 <= 0)
    throw JSONCompatible::InvalidJSON("reading kernel size", "receiving a negative number as size");

  kernel_size = cv::Size(j1, j2);
}


//  ██████╗ ██╗██╗      █████╗ ████████╗██╗ ██████╗ ███╗   ██╗
//  ██╔══██╗██║██║     ██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║
//  ██║  ██║██║██║     ███████║   ██║   ██║██║   ██║██╔██╗ ██║
//  ██║  ██║██║██║     ██╔══██║   ██║   ██║██║   ██║██║╚██╗██║
//  ██████╔╝██║███████╗██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║
//  ╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
//


void Dilation::apply(cv::Mat &img) const { //COMPLETE
  cv::dilate(img, img, cv::getStructuringElement(kernel_shape, kernel_size));
  //std::cout << "Applying Dilation" << std::endl;
  //printShit();
}

void Dilation::applyCopy(const cv::Mat &src, cv::Mat &dst) const { //COMPLETE
  cv::dilate(src, dst, cv::getStructuringElement(kernel_shape, kernel_size));
  //std::cout << "Applying CopyDilation" << std::endl;
  //printShit();
}


//  ███████╗██████╗  ██████╗ ███████╗██╗ ██████╗ ███╗   ██╗
//  ██╔════╝██╔══██╗██╔═══██╗██╔════╝██║██╔═══██╗████╗  ██║
//  █████╗  ██████╔╝██║   ██║███████╗██║██║   ██║██╔██╗ ██║
//  ██╔══╝  ██╔══██╗██║   ██║╚════██║██║██║   ██║██║╚██╗██║
//  ███████╗██║  ██║╚██████╔╝███████║██║╚██████╔╝██║ ╚████║
//  ╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝
//

void Erosion::apply(cv::Mat &img) const { //COMPLETE
  cv::erode(img, img, cv::getStructuringElement(kernel_shape, kernel_size));
  //std::cout << "Applying Erosion" << std::endl;
  //printShit();
}

void Erosion::applyCopy(const cv::Mat &src, cv::Mat &dst) const { //COMPLETE
  cv::erode(src, dst, cv::getStructuringElement(kernel_shape, kernel_size));
  //std::cout << "Applying CopyErosion" << std::endl;
  //printShit();
}

void Gaussian::apply(cv::Mat &img) const { //COMPLETE
  cv::GaussianBlur(img, img, kernel_size, sigma);
  if(threshold >= 0)
    cv::threshold(img, img, threshold, 255, cv::THRESH_BINARY);
  //std::cout << "Applying Erosion" << std::endl;
  //printShit();
}

void Gaussian::applyCopy(const cv::Mat &src, cv::Mat &dst) const { //COMPLETE
  cv::GaussianBlur(src, dst, kernel_size, sigma);
  if(threshold >= 0)
    cv::threshold(dst, dst, threshold, 255, cv::THRESH_BINARY);
  //std::cout << "Applying CopyErosion" << std::endl;
  //printShit();
}

void Gaussian::fromJSON(const json11::Json &json){
  auto arr = json.array_items();
  std::cout << "NEW JSON" << std::endl;

  if(arr.size() > 3){
    if(arr.size() >5)
      throw JSONCompatible::InvalidJSON("reading gaussian operation", "wrong number of parameters");

    if(arr[3].type() == json11::Json::NUMBER)
      sigma = arr[3].number_value();
    else
      throw JSONCompatible::InvalidJSON("reading gaussian sigma",  "not acceptable format");

    if(arr[4].type() == json11::Json::NUMBER)
      threshold = arr[4].int_value();
    else
      throw JSONCompatible::InvalidJSON("reading gaussian threshold",  "not acceptable format");
  } else {
    sigma = kernel_shape;
    threshold = -1;
  }
}

Gaussian::Gaussian(int shape, cv::Size size) : MorphologicalOperation(shape, size){
  if(shape != cv::MORPH_RECT || shape != cv::MORPH_CROSS || shape != cv::MORPH_ELLIPSE || size.height <= 0 || size.width <= 0){
    shape = -1;
    size = cv::Size(0,0);
  }
}

Gaussian::Gaussian(const json11::Json &json) : MorphologicalOperation(json){
  fromJSON(json);
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

//TODO: Test with other subclasses of MorphologicalOperation
//TODO: How to include gaussian in this shit?
/*
int main(){
  std::string err;
  json11::Json json = json11::Json::parse("{\"Processing\":[\"Erosion\", [2,222,222], \"n3\"]}", err);
  Dilation *d = (Dilation*) MorphologicalOperation::allocateFromJSON(json["Processing"], 0);
  cv::Mat m;
  if(d){
    d->printShit();
    d->apply(m);
  }
  delete d;
  return 0;
}
*/
