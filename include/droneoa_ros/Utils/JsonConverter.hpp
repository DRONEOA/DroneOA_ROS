#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <exception>
#include <json/value.h>
#include <jsoncpp/json/json.h>

class JsonConfig {
    std::vector<std::string> paramNames;
    std::unordered_map<std::string, std::string> params;

public:
    explicit JsonConfig(std::string filePath);
    virtual ~JsonConfig();
};
