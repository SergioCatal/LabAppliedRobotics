#ifndef IMAGE_OPERATION_HPP
#define IMAGE_OPERATION_HPP
#include"json.hpp"
#include <opencv2/opencv.hpp>

/**
 * Abstract class used to subclass objects defining operations which can be applied to a cv::Mat.
 */
class ImageOperation : public JSONCompatible{
public:
  
  /**
   * Virtual method used to apply the underlying operation to a cv::Mat image
   *
   * @param img input and output image for the operation
   */
  virtual void apply(cv::Mat &img) const = 0;

  /**
   * Virtual method used to apply the underlying operation to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operation
   * @param out output image for the operation
   */
  virtual void applyCopy(const cv::Mat &img, cv::Mat &out) const = 0;

  /**
   * Virtual destructor to allow call to custom destructor for subclasses
   */
   virtual ~ImageOperation(){};
};

#if 0
class GaussianBlur : public ImageOperation{
private:
  cv::Size
public:
  /**
   * Overridden method to read class members from a Json object
   *
   * @param json Json object from which to take kernel information
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the member variables of this object
   */
  json11::Json toJSON() const override;

  /**
   * Method used to apply the gaussian blur to a cv::Mat image
   *
   * @param img input and output image for the operation
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the gaussian blur to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operation
   * @param out output image for the operation
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;

};
#endif

#endif
