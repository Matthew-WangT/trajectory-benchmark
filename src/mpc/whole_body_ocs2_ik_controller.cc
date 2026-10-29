#include "mpc/whole_body_ocs2_ik_controller.h"

#include <glog/logging.h>

#include "mpc/dual_arm_ocs2_ik_controller.h"

namespace controller {

WholeBodyOcs2IkController::WholeBodyOcs2IkController(const std::string& urdf_file_path,
                                                     const Ocs2IkControllerConfig& config) {
  mpc_ = std::make_unique<Ocs2IkController>(urdf_file_path, config, config.info, "whole_body");
  CHECK_EQ(mpc_->ee_count(), 2) << config.info << " must name the left and the right end effector in eeFrames";
}

std::unique_ptr<IkController> MakeIkController(const std::string& urdf_file_path,
                                               const Ocs2IkControllerConfig& config) {
  if (config.type == "whole_body") {
    return std::make_unique<WholeBodyOcs2IkController>(urdf_file_path, config);
  }
  return std::make_unique<DualArmOcs2IkController>(urdf_file_path, config);
}

}  // namespace controller
