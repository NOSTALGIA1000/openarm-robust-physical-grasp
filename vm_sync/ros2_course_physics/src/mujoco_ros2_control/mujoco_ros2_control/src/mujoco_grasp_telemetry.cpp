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

#include "mujoco_ros2_control/mujoco_grasp_telemetry.hpp"

namespace mujoco_ros2_control
{

MujocoGraspTelemetry::MujocoGraspTelemetry(rclcpp::Node::SharedPtr &node) : node_(node) {}

void MujocoGraspTelemetry::init(const mjModel *mujoco_model)
{
  std::string object_geom;
  std::string gripper_body;

  if (node_->has_parameter("grasp_telemetry.object_geom"))
  {
    object_geom = node_->get_parameter("grasp_telemetry.object_geom").as_string();
  }
  if (node_->has_parameter("grasp_telemetry.gripper_body"))
  {
    gripper_body = node_->get_parameter("grasp_telemetry.gripper_body").as_string();
  }
  if (node_->has_parameter("grasp_telemetry.publish_every_n_steps"))
  {
    publish_every_n_steps_ =
      static_cast<int>(node_->get_parameter("grasp_telemetry.publish_every_n_steps").as_int());
  }
  if (publish_every_n_steps_ < 1)
  {
    publish_every_n_steps_ = 1;
  }

  if (object_geom.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "Grasp telemetry disabled (no grasp_telemetry.object_geom).");
    return;
  }

  object_geom_id_ = mj_name2id(mujoco_model, mjOBJ_GEOM, object_geom.c_str());
  if (object_geom_id_ < 0)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Grasp telemetry disabled: geom '%s' not found in model.",
      object_geom.c_str());
    return;
  }
  object_body_id_ = mujoco_model->geom_bodyid[object_geom_id_];

  if (gripper_body.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Grasp telemetry disabled: grasp_telemetry.gripper_body is empty.");
    return;
  }

  gripper_body_id_ = mj_name2id(mujoco_model, mjOBJ_BODY, gripper_body.c_str());
  if (gripper_body_id_ < 0)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Grasp telemetry disabled: body '%s' not found in model.",
      gripper_body.c_str());
    return;
  }

  publisher_ = node_->create_publisher<openarm_grasp_msgs::msg::GraspRawState>(
    "/grasp_raw_state", rclcpp::QoS(10));
  enabled_ = true;

  RCLCPP_INFO(
    node_->get_logger(), "Grasp telemetry enabled: geom '%s', body '%s', every %d steps.",
    object_geom.c_str(), gripper_body.c_str(), publish_every_n_steps_);
}

void MujocoGraspTelemetry::update(const mjModel *mujoco_model, mjData *mujoco_data)
{
  if (!enabled_)
  {
    return;
  }

  step_counter_++;
  if (step_counter_ < publish_every_n_steps_)
  {
    return;
  }
  step_counter_ = 0;

  openarm_grasp_msgs::msg::GraspRawState msg;
  msg.sim_time = mujoco_data->time;

  const mjtNum *object_pos = mujoco_data->xpos + 3 * object_body_id_;
  const mjtNum *object_quat = mujoco_data->xquat + 4 * object_body_id_;
  msg.object_pose.position.x = object_pos[0];
  msg.object_pose.position.y = object_pos[1];
  msg.object_pose.position.z = object_pos[2];
  msg.object_pose.orientation.w = object_quat[0];
  msg.object_pose.orientation.x = object_quat[1];
  msg.object_pose.orientation.y = object_quat[2];
  msg.object_pose.orientation.z = object_quat[3];

  const mjtNum *gripper_pos = mujoco_data->xpos + 3 * gripper_body_id_;
  const mjtNum *gripper_quat = mujoco_data->xquat + 4 * gripper_body_id_;
  msg.gripper_pose.position.x = gripper_pos[0];
  msg.gripper_pose.position.y = gripper_pos[1];
  msg.gripper_pose.position.z = gripper_pos[2];
  msg.gripper_pose.orientation.w = gripper_quat[0];
  msg.gripper_pose.orientation.x = gripper_quat[1];
  msg.gripper_pose.orientation.y = gripper_quat[2];
  msg.gripper_pose.orientation.z = gripper_quat[3];

  for (int i = 0; i < mujoco_data->ncon; ++i)
  {
    const mjContact &contact = mujoco_data->contact[i];
    int other_geom = -1;
    if (contact.geom[0] == object_geom_id_)
    {
      other_geom = contact.geom[1];
    }
    else if (contact.geom[1] == object_geom_id_)
    {
      other_geom = contact.geom[0];
    }
    else
    {
      continue;
    }

    mjtNum force[6] = {0, 0, 0, 0, 0, 0};
    mj_contactForce(mujoco_model, mujoco_data, i, force);

    const char *other_name = mj_id2name(mujoco_model, mjOBJ_GEOM, other_geom);
    msg.contact_geoms.push_back(other_name ? std::string(other_name) : std::string("unnamed"));
    msg.contact_normal_forces.push_back(static_cast<double>(force[0]));
  }

  publisher_->publish(msg);
}

}  // namespace mujoco_ros2_control
