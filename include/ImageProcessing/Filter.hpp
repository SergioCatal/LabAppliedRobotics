#ifndef FILTER_HPP
#define FILTER_HPP
#include "ImageOperation.hpp"

/**
 * Class to handle a filtering operations on an image.
 */
class Filter : public ImageOperation{
public:

  /**
   * Constructor that creates a filter from cv::Scalar inputs representing the lower and higher boundaries for the filter
   *
   * @param lows Lower boundaries for the filter
   * @param highs Higher boundaries for the filter
   */
  Filter(cv::Scalar ls, cv::Scalar hs);

  /**
   * Constructor that creates an filter from 6 inputs representing the lower and higher boundaries for the filter
   *
   * @param l1 lower boundary for the hue
   * @param l2 lower boundary for the saturation
   * @param l3 lower boundary for the value
   * @param h1 higher boundary for the hue
   * @param h2 higher boundary for the sturation
   * @param h3 higher boundary for the value
   */
  Filter(unsigned char l1, unsigned char l2, unsigned char l3, unsigned char h1, unsigned char h2, unsigned char h3);

  /**
   * Constructor to create a filter object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take information
   *
   * @throws InvalidJSON
   */
  Filter(const json11::Json &json);

  /**
   * Overridden method to fill the lower and higher boundaries from a Json object
   *
   * @param json Json object from which to take information about the boundaries
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the filter boundaries
   */
  json11::Json toJSON() const override;

protected:
  cv::Scalar lows, highs;
};


class InRangeFilter : public Filter {
public:
  /**
   * Constructor that creates a filter from cv::Scalar inputs representing the lower and higher boundaries for the filter
   *
   * @param lows Lower boundaries for the filter
   * @param highs Higher boundaries for the filter
   */
  InRangeFilter(cv::Scalar ls, cv::Scalar hs) : Filter(ls, hs){}

  /**
   * Constructor that creates an filter from 6 inputs representing the lower and higher boundaries for the filter
   *
   * @param l1 lower boundary for the hue
   * @param l2 lower boundary for the saturation
   * @param l3 lower boundary for the value
   * @param h1 higher boundary for the hue
   * @param h2 higher boundary for the sturation
   * @param h3 higher boundary for the value
   */
  InRangeFilter(unsigned char l1, unsigned char l2, unsigned char l3, unsigned char h1, unsigned char h2, unsigned char h3) : Filter(l1,l2,l3,h1,h2,h3){}

  /**
   * Constructor to create a filter object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take information
   *
   * @throws InvalidJSON
   */
  InRangeFilter(const json11::Json &json) : Filter(json){}

  /**
   * Method used to apply the filter to a cv::Mat image
   *
   * @param img input and output image for the operations
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the filter to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operations
   * @param out output image for the operations
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;
};

class AdaptiveFilter : public Filter {
public:
  /**
   * Constructor that creates a filter from cv::Scalar inputs representing the lower and higher boundaries for the filter
   *
   * @param lows Lower boundaries for the filter
   * @param highs Higher boundaries for the filter
   */
  AdaptiveFilter(cv::Scalar ls, cv::Scalar hs) : Filter(ls, hs){}

  /**
   * Constructor that creates an filter from 6 inputs representing the lower and higher boundaries for the filter
   *
   * @param l1 lower boundary for the hue
   * @param l2 lower boundary for the saturation
   * @param l3 lower boundary for the value
   * @param h1 higher boundary for the hue
   * @param h2 higher boundary for the sturation
   * @param h3 higher boundary for the value
   */
  AdaptiveFilter(unsigned char l1, unsigned char l2, unsigned char l3, unsigned char h1, unsigned char h2, unsigned char h3) : Filter(l1,l2,l3,h1,h2,h3){}

  /**
   * Constructor to create a filter object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take information
   *
   * @throws InvalidJSON
   */
  AdaptiveFilter(const json11::Json &json) : Filter(json){}

  /**
   * Method used to apply the filter to a cv::Mat image
   *
   * @param img input and output image for the operations
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the filter to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operations
   * @param out output image for the operations
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;
};

class TwoRangesFilter : public ImageOperation {
public:
  /**
   * Constructor that creates a filter from cv::Scalar inputs representing the lower and higher boundaries for the filter
   *
   * @param lows Lower boundaries for the filter
   * @param highs Higher boundaries for the filter
   */
  TwoRangesFilter(cv::Scalar ls1, cv::Scalar hs1, cv::Scalar ls2, cv::Scalar hs2);

  /**
   * Constructor to create a filter object directly from a Json object. It basically calls the fromJSON method
   *
   * @param json Json object from which to take information
   *
   * @throws InvalidJSON
   */
  TwoRangesFilter(const json11::Json &json);

  /**
   * Overridden method to fill the lower and higher boundaries from a Json object
   *
   * @param json Json object from which to take information about the boundaries
   *
   * @throws InvalidJSON
   */
  void fromJSON(const json11::Json &json) override;

  /**
   * Overridden method to write class members to a Json object
   *
   * @return Returns a Json object containing the filter boundaries
   */
  json11::Json toJSON() const override;

  /**
   * Method used to apply the filter to a cv::Mat image
   *
   * @param img input and output image for the operations
   */
  void apply(cv::Mat &img) const override;

  /**
   * Method used to apply the filter to an input cv::Mat image and output it on a second cv::Mat
   *
   * @param img input image for the operations
   * @param out output image for the operations
   */
  void applyCopy(const cv::Mat &src, cv::Mat &dst) const override;
private:
  InRangeFilter f1, f2;
};

#endif
