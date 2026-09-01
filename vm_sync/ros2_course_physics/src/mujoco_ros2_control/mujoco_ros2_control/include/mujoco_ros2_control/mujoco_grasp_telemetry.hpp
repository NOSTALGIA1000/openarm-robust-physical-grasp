#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_GRASP_TELEMETRY_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_GRASP_TELEMETRY_HPP_

#include <string>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "openarm_grasp_msgs/msg/grasp_raw_state.hpp"

namespace mujoco_ros2_control
{
// Read-only sampler of MuJoCo ground truth for physics-mode grasp evaluation.
// Publishes raw object contacts and poses; it performs no classification.
// Disabled unless grasp_telemetry.object_geom names an existing geom, so the
// original demo launch path is unaffected.
class MujocoGraspTelemetry
{
public:
  explicit MujocoGraspTelemetry(rclcpp::Node::SharedPtr &node);

  void init(const mjModel *mujoco_model);
  void update(const mjModel *mujoco_model, mjData *mujoco_data);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<openarm_grasp_msgs::msg::GraspRawState>::SharedPtr publisher_;

  bool enabled_{false};
  int object_geom_id_{-1};
  int object_body_id_{-1};
  int gripper_body_id_{-1};
  int publish_every_n_steps_{10};
  int step_counter_{0};
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_GRASP_TELEMETRY_HPP_
