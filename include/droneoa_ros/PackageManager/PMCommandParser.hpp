#ifndef PM_CMD_PARSER_
#define PM_CMD_PARSER_

#include <string>
#include <vector>
#include <map>

namespace PM {

struct PackageRecord {
    std::string name;
    std::string url;
    std::string branch;
    bool startWithMainNode;
    PackageRecord();
    PackageRecord(std::string name);
    std::string toString();
};

class CommandParser {
    std::string DRONEOA_PATH;
    std::map<std::string, PackageRecord> mPackageList;
    void install(std::vector<std::string> tokens);
    void update(std::vector<std::string> tokens);
    void uninstall(std::vector<std::string> tokens);
    void list(std::vector<std::string> tokens);
    bool rebuild();
    void launch();
    void shutdown(bool killAllNodes);
    void printHelp();
    // File Operation
    bool writeListToFile();
    bool readListFromFile();
 public:
    CommandParser();
    ~CommandParser();
    bool parseInput(std::string input);
    bool parseInput(std::vector<std::string> tokens);
};

}  // namespace PM

#endif
