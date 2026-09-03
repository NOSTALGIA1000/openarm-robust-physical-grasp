#include <cmath>

#include "mujoco_ros2_control/mujoco_scene_reset.hpp"

namespace mujoco_ros2_control
{

MujocoSceneReset::MujocoSceneReset(rclcpp::Node::SharedPtr &node) : node_(node) {}

void MujocoSceneReset::init(const mjModel *mujoco_model, mjData *mujoco_data)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  std::string geom_name;
  if (node_->has_parameter("grasp_telemetry.object_geom"))
  {
    geom_name = node_->get_parameter("grasp_telemetry.object_geom").as_string();
  }
  if (geom_name.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "Scene reset disabled (no object geom configured).");
    return;
  }

  int geom_id = mj_name2id(mujoco_model, mjOBJ_GEOM, geom_name.c_str());
  if (geom_id < 0)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Scene reset disabled: geom '%s' not found.", geom_name.c_str());
    return;
  }
  int body_id = mujoco_model->geom_bodyid[geom_id];
  int joint_id = mujoco_model->body_jntadr[body_id];
  if (joint_id < 0 || mujoco_model->jnt_type[joint_id] != mjJNT_FREE)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Scene reset disabled: body owning '%s' has no free joint, so its pose is not "
      "independently settable.",
      geom_name.c_str());
    return;
  }

  qpos_adr_ = mujoco_model->jnt_qposadr[joint_id];
  dof_adr_ = mujoco_model->jnt_dofadr[joint_id];
  for (int i = 0; i < 7; ++i)
  {
    spawn_[i] = mujoco_model->qpos0[qpos_adr_ + i];
  }

  service_ = node_->create_service<openarm_grasp_msgs::srv::ResetObject>(
    "/reset_object",
    std::bind(&MujocoSceneReset::handle, this, std::placeholders::_1, std::placeholders::_2));
  enabled_ = true;

  RCLCPP_INFO(
    node_->get_logger(), "Scene reset enabled on /reset_object (spawn %.4f %.4f %.4f).",
    spawn_[0], spawn_[1], spawn_[2]);
}

void MujocoSceneReset::handle(
  const std::shared_ptr<openarm_grasp_msgs::srv::ResetObject::Request> request,
  std::shared_ptr<openarm_grasp_msgs::srv::ResetObject::Response> response)
{
  if (!enabled_)
  {
    response->success = false;
    response->message = "scene reset not enabled";
    return;
  }

  // Position: spawn height is always reused; only the horizontal placement and
  // yaw are configurable, since a trial varies where the object sits on the
  // table rather than how far above it.
  double x = request->use_spawn ? spawn_[0] : request->x;
  double y = request->use_spawn ? spawn_[1] : request->y;
  double yaw = request->use_spawn ? 0.0 : request->yaw_deg * M_PI / 180.0;

  mj_data_->qpos[qpos_adr_ + 0] = x;
  mj_data_->qpos[qpos_adr_ + 1] = y;
  mj_data_->qpos[qpos_adr_ + 2] = spawn_[2];
  mj_data_->qpos[qpos_adr_ + 3] = std::cos(yaw / 2.0);
  mj_data_->qpos[qpos_adr_ + 4] = 0.0;
  mj_data_->qpos[qpos_adr_ + 5] = 0.0;
  mj_data_->qpos[qpos_adr_ + 6] = std::sin(yaw / 2.0);

  // Clear momentum, otherwise the object keeps whatever velocity the previous
  // trial left it with and drifts out of the pose just written.
  for (int i = 0; i < 6; ++i)
  {
    mj_data_->qvel[dof_adr_ + i] = 0.0;
    mj_data_->qacc[dof_adr_ + i] = 0.0;
  }

  mj_forward(mj_model_, mj_data_);

  response->success = true;
  response->sim_time = mj_data_->time;
  response->message = "object reset";
  RCLCPP_INFO(
    node_->get_logger(), "Object reset to (%.4f, %.4f) yaw %.1f deg at sim_time %.3f",
    x, y, request->use_spawn ? 0.0 : request->yaw_deg, mj_data_->time);
}

}  // namespace mujoco_ros2_control
