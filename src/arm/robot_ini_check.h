#pragma once

#include <string>

namespace arm {

// Checks the robot.ini settings the benchmark relies on in the [R.A0.BASIC] and [R.A1.BASIC] sections:
// PosDTType=1 and JointPIDCtlType=1, the latter enabling the PD velocity feedforward. Returns false with the
// reason in `error` otherwise.
bool CheckRobotIniText(const std::string& text, std::string& error);

}  // namespace arm
