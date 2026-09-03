#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SCENE_RESET_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SCENE_RESET_HPP_

#include <array>
#include <string>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "openarm_grasp_msgs/srv/reset_object.hpp"

namespace mujoco_ros2_control
{
// Restores the tracked object to a known pose in place, so a batch of trials can
// share one simulator instance instead of paying a full relaunch each time.
//
// Only the object is reset. The arm is left alone on purpose: its joints are
// force-controlled, so writing qpos would fight the running controllers, and the
// picker already drives the arm back to hover at the start of every trial.
//
// Simulation time is never reset. The trial validity guard treats a backward
// jump in sim_time as evidence of a diverged, silently auto-reset simulation;
// resetting time here would forge exactly that signature.
class MujocoSceneReset
{
public:
  explicit MujocoSceneReset(rclcpp::Node::SharedPtr &node);

  void init(const mjModel *mujoco_model, mjData *mujoco_data);

private:
  void handle(
    const std::shared_ptr<openarm_grasp_msgs::srv::ResetObject::Request> request,
    std::shared_ptr<openarm_grasp_msgs::srv::ResetObject::Response> response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<openarm_grasp_msgs::srv::ResetObject>::SharedPtr service_;

  const mjModel *mj_model_{nullptr};
  mjData *mj_data_{nullptr};
  bool enabled_{false};
  int qpos_adr_{-1};
  int dof_adr_{-1};
  std::array<double, 7> spawn_{};
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SCENE_RESET_HPP_
