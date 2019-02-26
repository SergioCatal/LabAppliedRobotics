#ifndef MORPHOLOGICAL_OPERATION_HPP
#define MORPHOLOGICAL_OPERATION_HPP
#include"ImageOperation.hpp"
#include <unordered_map>
#include <exception>
#include <functional>
#include <iostream> //DEBUG

/**
 * Class that handles the morphological operations on images.
 */
class MorphologicalOperation : public ImageOperation {
public:
  /**
   * Constructor to create an operation object when the kernel is already known.
   * It checks that the shape and size are acceptable values and sets shape to -1 otherwise
   *
   * @param shape the shape of the kernel. It must be among the following: cv::MORPH_RECT, cv::MORPH_CROSS, cv::MORPH_ELLIPSE
   * @param size the size of the kernel
   */
  MorphologicalOperation(int shape, cv::Size size);

  /**
   * Constructor to create the operation object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take kernel information
   *
   * @throws InvalidJSON
   */
  MorphologicalOperation(const json11::Json &json);

  /**
   * Overridden method to read class members from a Json object
   *
   * @param json Json object from which to take kernel information
   *
   * @throws InvalidJSON
   */
  virtual void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the member variables of this object
   */
  json11::Json toJSON() const override;

  /**
   * Static class used to allocate dynamically a MorphologicalOperation. The subclass actually allocated is chosen based on the json input
   *
   * @param json Json array to read class members from
   * @param i index in the json array to start from to read the data
   * @return a dynamically allocated subclass
   *
   * @throws InvalidJSON
   */
  static MorphologicalOperation* allocateFromJSON(const json11::Json &json, int i);

  /*
   * Debug method
   */
  void printShit() const {std::cout << kernel_shape << std::endl;}

  /**
   * Virtual destructor to allow call to custom destructor for subclasses
   */
  virtual ~MorphologicalOperation(){};

protected:
  int kernel_shape;
  cv::Size kernel_size;
  static const std::unordered_map<std::string, int> encode_kernel;
  static const std::unordered_map<std::string, std::function<MorphologicalOperation*(const json11::Json &json)>> encode_operation_type;

private:

  /**
   * Method for obtaining the kernel shape from a json object.
   *
   * @param json Json object to read from
   * @throws InvalidJSON
   */
  void readKernelShape(const json11::Json &json);

  /**
   * Method for obtaining the kernel size from a json object.
   *
   * @param json Json object to read from
   * @throws InvalidJSON
   */
  void readKernelSize(const json11::Json &json1, const json11::Json &json2);

};

/**
 * Class that handles the dilation operation on images
 */
class Dilation : public MorphologicalOperation{
public:
  /**
   * Statements needed to explicitly use the parent's constructors
   */
  using MorphologicalOperation::MorphologicalOperation;

  /**
   * Method used to apply the dilation operation to a cv::Mat image
   *
   * @param img input and output image for the operation
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the dilation operation to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operation
   * @param out output image for the operation
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;
};

/**
 * Class that handles the erosion operation on images
 */
class Erosion : public MorphologicalOperation{
public:
  /**
   * Statements needed to explicitly use the parent's constructors
   */
  using MorphologicalOperation::MorphologicalOperation;

  /**
   * Method used to apply the erosion operation to a cv::Mat image
   *
   * @param img input and output image for the operation
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the erosion operation to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operation
   * @param out output image for the operation
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;
};

/**
 * Class that handles the erosion operation on images
 */
class Gaussian : public MorphologicalOperation{
public:
   /**
    * Constructor to create an operation object when the kernel is already known.
    * It checks that the shape and size are acceptable values and sets shape to -1 otherwise
    *
    * @param shape the shape of the kernel. It must be among the following: cv::MORPH_RECT, cv::MORPH_CROSS, cv::MORPH_ELLIPSE
    * @param size the size of the kernel
    */
   Gaussian(int shape, cv::Size size);

   /**
    * Constructor to create the operation object directly from a Json object. It basically calls the fromJSON method
    *
    * @param json Json object from which to take kernel information
    *
    * @throws InvalidJSON
    */
   Gaussian(const json11::Json &json);

  /**
   * Method used to apply the erosion operation to a cv::Mat image
   *
   * @param img input and output image for the operation
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the erosion operation to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operation
   * @param out output image for the operation
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;

  /**
   * Overridden method to read class members from a Json object
   *
   * @param json Json object from which to take kernel information
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;
private:
  double sigma;
  int threshold;
};


#endif
