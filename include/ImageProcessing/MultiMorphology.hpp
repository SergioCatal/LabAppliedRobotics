#ifndef MULTI_MORPHOLOGY_HPP
#define MULTI_MORPHOLOGY_HPP
#include "ImageOperation.hpp"
#include "MorphologicalOperation.hpp"


/**
 * Class to handle a series of operations on an image. The description of such operations comes from a JSON file
 */
class MultiMorphology : public ImageOperation{
public:

  /**
   * Default constructor that creates an object with null values i.e. empty vector operations
   */
  MultiMorphology();

  /**
   * Constructor to create the multi-operation object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take information
   *
   * @throws InvalidJSON
   */
  MultiMorphology(const json11::Json &json);

  /**
   * Destructor that deallocates the dynamically-allocated objects in the vector operations
   */
  ~MultiMorphology();

  /**
   * Overridden method to fill the vector @operations from a Json object
   *
   * @param json Json object from which to take information about the operations to perform
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the operations in the vector @operations
   */
  json11::Json toJSON() const override;

  /**
   * Method used to apply the series of operations in the vector @operations to a cv::Mat image
   *
   * @param img input and output image for the operations
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the series of operations in the vector @operations to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operations
   * @param out output image for the operations
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;

private:
  std::vector<MorphologicalOperation*> operations;
};

#endif
