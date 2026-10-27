#include "arm/robot_ini_check.h"

#include <array>
#include <map>
#include <sstream>
#include <utility>

namespace arm {
namespace {

std::string Trim(const std::string& s) {
  const size_t begin = s.find_first_not_of(" \t\r");
  if (begin == std::string::npos) {
    return "";
  }
  const size_t end = s.find_last_not_of(" \t\r");
  return s.substr(begin, end - begin + 1);
}

}  // namespace

bool CheckRobotIniText(const std::string& text, std::string& error) {
  constexpr std::array<const char*, 2> kSections = {"R.A0.BASIC", "R.A1.BASIC"};
  constexpr std::array<std::pair<const char*, const char*>, 2> kExpected = {{
      {"PosDTType", "1"},
      {"JointPIDCtlType", "1"},
  }};

  std::map<std::string, std::map<std::string, std::string>> values;
  std::istringstream stream(text);
  std::string line;
  std::string section;
  while (std::getline(stream, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == ';' || line.front() == '#') {
      continue;
    }
    if (line.front() == '[' && line.back() == ']') {
      section = line.substr(1, line.size() - 2);
      continue;
    }
    const size_t eq = line.find('=');
    if (eq != std::string::npos) {
      values[section][Trim(line.substr(0, eq))] = Trim(line.substr(eq + 1));
    }
  }

  error.clear();
  for (const char* section_name : kSections) {
    for (const auto& [key, expected] : kExpected) {
      const auto section_it = values.find(section_name);
      std::string actual = "<missing>";
      if (section_it != values.end()) {
        const auto value_it = section_it->second.find(key);
        if (value_it != section_it->second.end()) {
          actual = value_it->second;
        }
      }
      if (actual != expected) {
        error += std::string("[") + section_name + "] " + key + "=" + actual + ", expected " + expected + "; ";
      }
    }
  }
  return error.empty();
}

}  // namespace arm
