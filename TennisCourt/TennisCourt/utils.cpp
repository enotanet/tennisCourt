#include "utils.h"
#include <map>
#include <vector>
#include <string>
#include <sstream>

std::map<std::string, std::vector<std::string>> g_args;

// Very hacky and unsafe. Tests?
//
void ParseArguments(int argc, char *argv[]) {
  for (int i = 0; i < argc; ++i) {
    std::stringstream sin(argv[i]);
    std::string key;
    std::getline(sin, key, '=');
    if (!sin.eof()) {
      if (key.size() > 2 && key.substr(0, 2) == "--") {
        key = key.substr(2);
        std::string value;
        while (!sin.eof()) {
          getline(sin, value, ';');
          g_args[key].push_back(value);
        }
      }
    }
  }
}