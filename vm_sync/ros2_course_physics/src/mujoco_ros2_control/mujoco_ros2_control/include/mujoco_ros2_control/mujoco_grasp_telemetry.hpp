// Copyright 2026 the openarm-robust-physical-grasp authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Added to the MIT-licensed mujoco_ros2_control package (Copyright (c) 2025
// Sangtaek Lee); this file is not derived from it. See NOTICE at the repo root.

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
  // The two finger collision geoms. Their midpoint is where the jaws meet,
  // which is what an offset has to be measured from if it is to mean the same
  // thing in different approach poses - the link origin does not.
  int left_finger_geom_id_{-1};
  int right_finger_geom_id_{-1};
  int publish_every_n_steps_{10};
  int step_counter_{0};
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_GRASP_TELEMETRY_HPP_
