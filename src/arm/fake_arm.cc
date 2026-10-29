#include "arm/fake_arm.h"

#include <cmath>

namespace arm {

FakeArm::FakeArm(const JointVector& left_q0, const JointVector& right_q0, double time_constant_s)
    : time_constant_s_(time_constant_s),
      left_cmd_(left_q0),
      right_cmd_(right_q0),
      start_(std::chrono::steady_clock::now()) {
  state_.left.q = state_.left.q_ext = state_.left.q_cmd_echo = left_q0;
  state_.right.q = state_.right.q_ext = state_.right.q_cmd_echo = right_q0;
}

bool FakeArm::Send(const JointVector& left_q, const JointVector& right_q) {
  left_cmd_ = left_q;
  right_cmd_ = right_q;
  ++sent_frames_;
  state_.left.in_frame_serial = state_.right.in_frame_serial = sent_frames_;
  return true;
}

void FakeArm::Integrate(ArmFeedback& arm, const JointVector& cmd, double dt) const {
  // A zero time constant is the ideal robot of the trajectory solve, the measurement is the command.
  const double alpha = time_constant_s_ > 0.0 ? 1.0 - std::exp(-dt / time_constant_s_) : 1.0;
  const JointVector q_next = arm.q + alpha * (cmd - arm.q);
  arm.dq = (q_next - arm.q) / dt;
  arm.q = q_next;
  arm.q_ext = q_next;
  arm.q_cmd_echo = q_next;
  arm.q_cmd_received = cmd;
}

bool FakeArm::Read(Feedback& feedback) {
  const auto elapsed = std::chrono::steady_clock::now() - start_;
  const int64_t frame = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  state_.is_new = frame != last_frame_;
  if (state_.is_new) {
    const double dt = last_frame_ < 0 ? 1e-3 : 1e-3 * static_cast<double>(frame - last_frame_);
    Integrate(state_.left, left_cmd_, dt);
    Integrate(state_.right, right_cmd_, dt);
    last_frame_ = frame;
  }
  state_.serial = frame;
  feedback = state_;
  return true;
}

}  // namespace arm
