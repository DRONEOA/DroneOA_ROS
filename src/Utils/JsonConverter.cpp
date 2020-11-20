// get object from json
// check whether new config param
// update value based on changes

#include <droneoa_ros/Utils/JsonConverter.hpp>

JsonConfig::JsonConfig(std::string filePath) {
    Json::Value config;
    try {
        std::ifstream ifs(filePath);          
        ifs >> config;
        std::cout << config;
    }
    catch (std::exception e) {
        std::cout << e.what() << std::endl;
    }
    paramNames.emplace_back("maxHeight");
    for (auto parameter : paramNames) {
        params[parameter] = config[parameter].asString();
        std::cout << parameter << ": " << params[parameter] << std::endl;
    }
}

JsonConfig::~JsonConfig(){}

