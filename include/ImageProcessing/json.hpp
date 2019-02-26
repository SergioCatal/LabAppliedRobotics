#ifndef JSON_HPP
#define JSON_HPP
#include"json11.hpp"
#include<exception>
#include<string>
#include<fstream>
#include <sstream>
#include <iostream>

/**
 * Abstract class used to subclass objects which can be written and read to/from a JSON file.
 */
class JSONCompatible{
public:
  /**
   * Exception class thrown when the json file read is not correctly parsed
   */
  class InvalidJSON : public std::exception {
    std::string invalid_section, invalid_part;
  public:
    /**
     * Constructor to save in a private member the part that triggerd this ex
     *
     * @param invalid_section contains information about the wrong section of the JSON
     * @param invalid_part contains information about the wrong part of the JSON in the invalid_section
     */
    InvalidJSON(std::string invalid_section, std::string invalid_part);

    /**
     * Method called to obtain information about this exception. The returned message will specify the kind of exception and the value that triggered it
     */
    const char* what() const throw() override;
  };

  /**
   * Virtual method that defines the interface to read class members from a JSON object.
   *
   * @param json Json object from which to take kernel information
   */
  virtual void fromJSON(const json11::Json &json) = 0;

  /**
   * Virtual method that defines the interface to write class members to a JSON object.
   */
  virtual json11::Json toJSON() const = 0;

  /**
   * Virtual destructor to allow call to custom destructor for subclasses
   */
   virtual ~JSONCompatible(){};


  inline static json11::Json element_of(const json11::Json json, const std::string element_name){
    json11::Json ret = json[element_name];
    if(ret.type() == json11::Json::NUL)
      throw InvalidJSON("looking for element", "missing required element " + element_name);
    return ret;
  }

  inline static json11::Json& read_json(json11::Json &j, std::string path){
    std::string err;
    std::ifstream f(path);
    if(!f)
      std::cout << "NOT ABLE TO READ FILE" <<std::endl;
    else {
      std::stringstream buf;
      buf << f.rdbuf();
      std::string json_string = buf.str();
      j = json11::Json::parse(json_string, err);
    }
    return j;
  }
};


#endif
