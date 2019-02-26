#include"json.hpp"

JSONCompatible::InvalidJSON::InvalidJSON(std::string inv_s, std::string inv_p) : invalid_section(inv_s), invalid_part(inv_p){
}

const char* JSONCompatible::InvalidJSON::what() const throw(){
  return ("Invalid JSON file. While " + invalid_section + " found an error for " + invalid_part).c_str();
}


/* TEST MAIN */
/*
#include<iostream>
int main(){
  std::string err;
  json11::Json json = json11::Json::parse("{\"names\":[\"n1\", \"n2\", \"n3\"]}", err);
  std::cout << json["name"].array_items().size() << std::endl;
  return 0;
}
*/
