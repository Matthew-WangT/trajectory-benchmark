#include "arm/robot_ini_check.h"

#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

namespace arm {
namespace {

std::string ReadSample() {
  std::ifstream file(std::string(TEST_DATA_DIR) + "/robot_infer06.ini");
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

std::string Replace(std::string text, const std::string& from, const std::string& to) {
  const size_t pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << from;
  return text.replace(pos, from.size(), to);
}

TEST(RobotIniCheckTest, AcceptsTheReferenceRobot) {
  std::string error;
  EXPECT_TRUE(CheckRobotIniText(ReadSample(), error)) << error;
  EXPECT_TRUE(error.empty());
}

TEST(RobotIniCheckTest, RejectsAWrongFlag) {
  // The first occurrence is the left arm, [R.A0.BASIC].
  const std::string text = Replace(ReadSample(), "JointPIDCtlType=1", "JointPIDCtlType=0");
  std::string error;
  EXPECT_FALSE(CheckRobotIniText(text, error));
  EXPECT_NE(error.find("[R.A0.BASIC] JointPIDCtlType=0"), std::string::npos) << error;
}

TEST(RobotIniCheckTest, RejectsAMissingSection) {
  const std::string text = Replace(ReadSample(), "[R.A1.BASIC]", "[R.A1.OTHER]");
  std::string error;
  EXPECT_FALSE(CheckRobotIniText(text, error));
  EXPECT_NE(error.find("[R.A1.BASIC] PosDTType=<missing>"), std::string::npos) << error;
}

TEST(RobotIniCheckTest, IgnoresTheFlagsOfOtherSections) {
  const std::string text = "[R.A0.BASIC]\nPosDTType=1\nJointPIDCtlType=1\n[R.A0.CTRL]\nPosDTType=0\n"
                           "[R.A1.BASIC]\r\nPosDTType = 1\r\nJointPIDCtlType=1\r\n";
  std::string error;
  EXPECT_TRUE(CheckRobotIniText(text, error)) << error;
}

}  // namespace
}  // namespace arm
