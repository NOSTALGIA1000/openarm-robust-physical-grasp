# OpenArm ROS2 Physical Grasp — Stage 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the course demo's software-attached object motion with a measurable MuJoCo contact-dynamics grasp: the left gripper closes under bounded force, lifts a free rigid body `20 mm`, and holds it `2.0 s` with bilateral finger contact, all evidenced by simulator ground-truth telemetry.

**Architecture:** A new `physics_mode` runs on separate model, URDF, SRDF, controller-config, and launch files so the original `demo_mode` path stays byte-identical. The MuJoCo node gains a thin C++ telemetry publisher that emits raw object/gripper poses and raw object contacts; all classification, state-machine, and acceptance logic lives in a new pure-Python ROS2 package that is unit-testable without a simulator. The left finger joint switches from direct `qpos` teleportation to bounded PID force via the plugin's existing PID branch; arm joints keep the current MoveIt position path (Stage 3 changes those).

**Tech Stack:** ROS2 Humble, MuJoCo 3.10.0, `ros2_control` / `controller_manager`, MoveIt 2, `control_toolbox::Pid`, Python 3.10.12, pytest 6.2.5, colcon.

## Global Constraints

- VM access is SSH only. Never start, stop, or control the VM desktop: `ssh -p 2224 nostalgia@127.0.0.1`, `scp -P 2224`.
- Course workspace (the only ROS2 workspace this plan may modify): `/home/nostalgia/ros2_ws_course_repro/ros2_ws`.
- **Never edit `/home/nostalgia/openarm_ws`.**
- Build base and install base are non-default and must be passed every time: `--build-base build_repro --install-base install_repro`.
- Build command template:
  ```bash
  cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select PKG
  ```
- Runtime source command template:
  ```bash
  source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71
  ```
- `ROS_DOMAIN_ID=71` for every ROS2 command in this plan.
- MuJoCo physics timestep stays `0.002`. Physics semantics must never be changed to improve apparent speed.
- `demo_mode` files that must remain unmodified: `model/table_scene.xml`, `model/table_scene_full.urdf`, `model/openarm_bimanual.xml`, `config/openarm.srdf`, `config/ros2_controllers.yaml`, `launch/my_controllers.yaml`, `launch/run_simulation_with_detector.launch.py`, `launch/bringup_all.launch.py`, `launch/move_group.launch.py`, `launch/pick_apple.py`.
- Windows research repo (git lives here, not on the VM): `C:\Users\ASUS\Documents\ChatGPT\open-arm相关`.
- VM→Windows mirror directory for all course-workspace changes: `vm_sync/ros2_course_physics/`.
- Canonical names used across every task (do not rename):
  - object geom: `physics_object_geom`; object body: `physics_object`; object freejoint: `physics_object_free`
  - left finger geoms: `openarm_left_left_finger_collision`, `openarm_left_right_finger_collision`
  - gripper reference body: `openarm_left_hand`
  - telemetry topic: `/grasp_raw_state`
  - telemetry message: `openarm_grasp_msgs/msg/GraspRawState`
- Declared Stage 1 thresholds (verbatim from the design spec, plus two declared parameters the spec left open):
  - lift `0.020 m`, hold `2.0 s`, bilateral hold ratio `0.80`, drift `0.010 m`
  - declared: contact normal-force threshold `0.5 N`, bilateral confirmation window `0.30 s`
- Acceptance run: 10 trials in a fixed deterministic condition, at least 8 successes.

---

## File Structure

**New package `openarm_grasp_msgs`** (`src/openarm_grasp_msgs/`) — one raw-telemetry message. Kept separate so the C++ plugin package and the Python logic package share a type without depending on each other.

**Modified package `mujoco_ros2_control`** — three narrow changes:
- `include/mujoco_ros2_control/mujoco_grasp_telemetry.hpp`, `src/mujoco_grasp_telemetry.cpp` (new): read-only MuJoCo contact/pose sampler and publisher. Disabled unless a parameter names the object geom, so demo runs are unaffected.
- `src/mujoco_system.cpp` + `include/.../mujoco_system.hpp`: clamp PID position output by a per-joint `position_max_effort` parameter.
- `src/mujoco_ros2_control_node.cpp`: instantiate telemetry; make viewer-render and camera periods parameters (defaults reproduce current behavior exactly).

**Modified package `openarm_moveit_config`** — physics-mode siblings only, no baseline file touched:
- `model/openarm_bimanual_physics.xml`, `model/table_scene_physics.xml`
- `model/table_scene_physics.urdf`
- `config/openarm_physics.srdf`
- `launch/physics_controllers.yaml`, `launch/move_group_physics.launch.py`, `launch/run_physics_grasp.launch.py`

**New package `openarm_grasp_physics`** (`src/openarm_grasp_physics/`) — all research logic, split by responsibility so each file is independently testable:
- `config.py` — thresholds and geom/frame names (frozen dataclass)
- `contact.py` — raw contact list → left/right/bilateral summary
- `geometry.py` — object pose relative to gripper, elevation, drift
- `evaluation.py` — failure-category precedence and success gate
- `trial_record.py` — sample accumulation, CSV/JSON/Markdown outputs
- `physics_picker.py` — ROS2 node and bounded grasp state machine
- `scripts/run_acceptance.sh` — 10-trial driver (launches a fresh simulator per trial, then aggregates results)
- `test/` — pytest suite for every pure module

---

### Task 1: Telemetry message package and VM↔Windows sync script

**Files:**
- Create (VM): `~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/package.xml`
- Create (VM): `~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/CMakeLists.txt`
- Create (VM): `~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/msg/GraspRawState.msg`
- Create (Windows): `tools/sync_from_vm.sh`

**Interfaces:**
- Consumes: nothing.
- Produces: message type `openarm_grasp_msgs/msg/GraspRawState` with fields `float64 sim_time`, `geometry_msgs/Pose object_pose`, `geometry_msgs/Pose gripper_pose`, `string[] contact_geoms`, `float64[] contact_normal_forces`. Every later task uses exactly these field names. Also produces `tools/sync_from_vm.sh`, used by the commit step of every later task.

- [ ] **Step 1: Create the package directory and message on the VM**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "mkdir -p ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/msg"
```

- [ ] **Step 2: Write `msg/GraspRawState.msg`**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/msg/GraspRawState.msg" <<'EOF'
# Simulator ground-truth grasp telemetry. Raw and unclassified on purpose:
# contact classification and grasp evaluation live in openarm_grasp_physics
# so they stay unit-testable without a simulator.

# MuJoCo simulation time in seconds (mj_data->time), not wall clock.
float64 sim_time

# World pose of the body owning the tracked object geom.
geometry_msgs/Pose object_pose

# World pose of the gripper reference body.
geometry_msgs/Pose gripper_pose

# For every MuJoCo contact involving the tracked object geom, the name of the
# other geom, and the normal component of that contact force in newtons.
# The two arrays always have the same length.
string[] contact_geoms
float64[] contact_normal_forces
EOF
```

- [ ] **Step 3: Write `package.xml`**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/package.xml" <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>openarm_grasp_msgs</name>
  <version>0.1.0</version>
  <description>Ground-truth grasp telemetry messages for OpenArm physics-mode grasping.</description>
  <maintainer email="3096954761qq@gmail.com">nostalgia</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>geometry_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

- [ ] **Step 4: Write `CMakeLists.txt`**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_msgs/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.5)
project(openarm_grasp_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspRawState.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOF
```

- [ ] **Step 5: Build the message package**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select openarm_grasp_msgs"
```
Expected: `Summary: 1 package finished`, no `Failed` line.

- [ ] **Step 6: Verify the interface is discoverable**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && ROS_DOMAIN_ID=71 ros2 interface show openarm_grasp_msgs/msg/GraspRawState"
```
Expected: the five fields printed, ending with `float64[] contact_normal_forces`.

- [ ] **Step 7: Write the mirror script on Windows**

Create `C:\Users\ASUS\Documents\ChatGPT\open-arm相关\tools\sync_from_vm.sh`:

```bash
#!/usr/bin/env bash
# Mirror physics-mode course-workspace files from the VM into the research repo.
# Run from the repo root. Read-only with respect to the VM.
set -euo pipefail

VM="nostalgia@127.0.0.1"
PORT=2224
WS="/home/nostalgia/ros2_ws_course_repro/ros2_ws"
DEST="vm_sync/ros2_course_physics"

PATHS=(
  "src/openarm_grasp_msgs"
  "src/openarm_grasp_physics"
  "src/mujoco_ros2_control/mujoco_ros2_control/CMakeLists.txt"
  "src/mujoco_ros2_control/mujoco_ros2_control/package.xml"
  "src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_grasp_telemetry.hpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_grasp_telemetry.cpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_node.cpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp"
  "src/openarm_moveit_config/model/openarm_bimanual_physics.xml"
  "src/openarm_moveit_config/model/table_scene_physics.xml"
  "src/openarm_moveit_config/model/table_scene_physics.urdf"
  "src/openarm_moveit_config/config/openarm_physics.srdf"
  "src/openarm_moveit_config/launch/physics_controllers.yaml"
  "src/openarm_moveit_config/launch/move_group_physics.launch.py"
  "src/openarm_moveit_config/launch/run_physics_grasp.launch.py"
)

for p in "${PATHS[@]}"; do
  if ssh -p "$PORT" "$VM" "test -e $WS/$p"; then
    mkdir -p "$DEST/$(dirname "$p")"
    scp -q -P "$PORT" -r "$VM:$WS/$p" "$DEST/$(dirname "$p")/"
    echo "synced $p"
  else
    echo "skip (absent on VM) $p"
  fi
done

# Never mirror build products.
find "$DEST" -type d \( -name '__pycache__' -o -name '.pytest_cache' \) -prune -exec rm -rf {} + 2>/dev/null || true
echo "mirror complete"
```

- [ ] **Step 8: Run the mirror script and verify it copied the message package**

Run:
```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && ls vm_sync/ros2_course_physics/src/openarm_grasp_msgs/msg
```
Expected: `synced src/openarm_grasp_msgs`, several `skip (absent on VM)` lines, and `GraspRawState.msg` listed.

- [ ] **Step 9: Commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && git add tools/sync_from_vm.sh vm_sync/ros2_course_physics && git commit -m "feat: add GraspRawState telemetry message and VM mirror script"
```

---

### Task 2: MuJoCo grasp telemetry publisher

**Files:**
- Create (VM): `src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_grasp_telemetry.hpp`
- Create (VM): `src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_grasp_telemetry.cpp`
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_node.cpp`
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/CMakeLists.txt`
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/package.xml`

All VM paths in this task are relative to `~/ros2_ws_course_repro/ros2_ws/`.

**Interfaces:**
- Consumes: `openarm_grasp_msgs/msg/GraspRawState` from Task 1.
- Produces: ROS2 parameters `grasp_telemetry.object_geom` (string, default `""` = disabled), `grasp_telemetry.gripper_body` (string, default `""`), `grasp_telemetry.publish_every_n_steps` (int, default `10`); topic `/grasp_raw_state`. Class `mujoco_ros2_control::MujocoGraspTelemetry` with `void init(mjModel *)` and `void update(const mjModel *, mjData *)`.

- [ ] **Step 1: Write the header**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_grasp_telemetry.hpp" <<'EOF'
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
EOF
```

- [ ] **Step 2: Write the implementation**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_grasp_telemetry.cpp" <<'EOF'
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
EOF
```

- [ ] **Step 3: Add the dependency to `package.xml`**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control && cp package.xml package.xml.pre_physics && sed -i 's#<depend>control_toolbox</depend>#<depend>control_toolbox</depend>\n  <depend>openarm_grasp_msgs</depend>#' package.xml && grep -n openarm_grasp_msgs package.xml"
```
Expected: one line containing `<depend>openarm_grasp_msgs</depend>`.

- [ ] **Step 4: Add the new source and dependency to `CMakeLists.txt`**

Back up, then add three entries: the `find_package`, the `THIS_PACKAGE_DEPENDS` entry, and the new source file in `add_executable(mujoco_ros2_control ...)`.

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control && cp CMakeLists.txt CMakeLists.txt.pre_physics"
```

Resulting `CMakeLists.txt` fragments:

```cmake
find_package(control_toolbox REQUIRED)
find_package(openarm_grasp_msgs REQUIRED)
```

```cmake
set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
  openarm_grasp_msgs
)
```

```cmake
add_executable(mujoco_ros2_control
  src/mujoco_ros2_control_node.cpp
  src/mujoco_rendering.cpp
  src/mujoco_ros2_control.cpp
  src/mujoco_cameras.cpp
  src/mujoco_grasp_telemetry.cpp
)
```

- [ ] **Step 5: Wire telemetry into the node main loop**

Back up first:

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/src && cp mujoco_ros2_control_node.cpp mujoco_ros2_control_node.cpp.pre_physics"
```

Apply three edits to `src/mujoco_ros2_control_node.cpp`.

Edit A — add the include next to the existing ones:

```cpp
#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_grasp_telemetry.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
```

Edit B — construct telemetry right after the cameras are initialized:

```cpp
  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);

  auto telemetry = std::make_unique<mujoco_ros2_control::MujocoGraspTelemetry>(node);
  telemetry->init(mujoco_model);
```

Edit C — sample telemetry on every physics step, inside the inner loop:

```cpp
    while (mujoco_data->time - simstart < 1.0 / 60.0)
    {
      mujoco_control.update();
      telemetry->update(mujoco_model, mujoco_data);
    }
```

- [ ] **Step 6: Build the plugin package**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && source install_repro/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select mujoco_ros2_control"
```
Expected: `Summary: 1 package finished`, no `Failed` line.

- [ ] **Step 7: Verify the demo path still runs and telemetry stays off**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && timeout 40 ros2 launch openarm_moveit_config run_simulation_with_detector.launch.py > /tmp/demo_telemetry_check.log 2>&1; grep -c 'Grasp telemetry disabled' /tmp/demo_telemetry_check.log"
```
Expected: `1`.

- [ ] **Step 8: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: publish MuJoCo ground-truth grasp telemetry from the simulation node"
```

---

### Task 3: Bounded PID effort for gripper position control

The plugin's PID branch writes `qfrc_applied` with no output limit, so a physics-mode gripper could apply unbounded force. This task adds an explicit per-joint clamp read from a URDF parameter. No joint in this workspace uses a `_pid` command interface today, so the baseline is unaffected.

**Files:**
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp`
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: URDF `ros2_control` joint parameter `position_max_effort` (newtons for a slide joint). When a joint declares a `position_pid` command interface, its PID output is clamped to `[-position_max_effort, +position_max_effort]`. Task 7's physics URDF sets it for `openarm_left_finger_joint1`.

- [ ] **Step 1: Back up both files**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control && cp include/mujoco_ros2_control/mujoco_system.hpp include/mujoco_ros2_control/mujoco_system.hpp.pre_physics && cp src/mujoco_system.cpp src/mujoco_system.cpp.pre_physics"
```

- [ ] **Step 2: Add the parameter-name constant and the struct field**

In `include/mujoco_ros2_control/mujoco_system.hpp`, extend the constant block:

```cpp
constexpr char PARAM_I_MIN[]{"_i_min"};
constexpr char PARAM_MAX_EFFORT[]{"_max_effort"};
```

and add one field to `struct JointState`, immediately after `bool is_pid_enabled{false};`:

```cpp
    bool is_pid_enabled{false};
    double pid_max_effort{std::numeric_limits<double>::max()};
```

- [ ] **Step 3: Confirm `<limits>` is available in the header**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "grep -n 'include <limits>' ~/ros2_ws_course_repro/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp"
```
If there is no output, add `#include <limits>` to the header's include block before building.

- [ ] **Step 4: Read the parameter when PID is enabled**

In `src/mujoco_system.cpp`, replace the PID-gain block at the end of `register_joints`:

```cpp
    // Get PID gains, if needed
    if (last_joint_state.is_pid_enabled)
    {
      last_joint_state.position_pid = get_pid_gains(joint, hardware_interface::HW_IF_POSITION);
      last_joint_state.velocity_pid = get_pid_gains(joint, hardware_interface::HW_IF_VELOCITY);
    }
```

with:

```cpp
    // Get PID gains, if needed
    if (last_joint_state.is_pid_enabled)
    {
      last_joint_state.position_pid = get_pid_gains(joint, hardware_interface::HW_IF_POSITION);
      last_joint_state.velocity_pid = get_pid_gains(joint, hardware_interface::HW_IF_VELOCITY);

      // Bound the force the PID branch may inject into MuJoCo. Without this the
      // controller can apply arbitrarily large joint forces, which would make a
      // "successful" grasp physically meaningless.
      const std::string max_effort_key =
        std::string(hardware_interface::HW_IF_POSITION) + std::string(PARAM_MAX_EFFORT);
      if (joint.parameters.find(max_effort_key) != joint.parameters.end())
      {
        last_joint_state.pid_max_effort = std::stod(joint.parameters.at(max_effort_key));
        RCLCPP_INFO_STREAM(
          logger_, "Joint " << last_joint_state.name << " PID effort limited to "
                            << last_joint_state.pid_max_effort << " N");
      }
    }
```

- [ ] **Step 5: Clamp the PID output in `write()`**

In `src/mujoco_system.cpp`, replace:

```cpp
      if (joint_state.is_pid_enabled)
      {
        double error = joint_state.position_command - mj_data_->qpos[joint_state.mj_pos_adr];
        mj_data_->qfrc_applied[joint_state.mj_vel_adr] =
          joint_state.position_pid.computeCommand(error, period.nanoseconds());
      }
```

with:

```cpp
      if (joint_state.is_pid_enabled)
      {
        double error = joint_state.position_command - mj_data_->qpos[joint_state.mj_pos_adr];
        double command = joint_state.position_pid.computeCommand(error, period.nanoseconds());
        mj_data_->qfrc_applied[joint_state.mj_vel_adr] =
          clamp(command, -joint_state.pid_max_effort, joint_state.pid_max_effort);
      }
```

- [ ] **Step 6: Build**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && source install_repro/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select mujoco_ros2_control"
```
Expected: `Summary: 1 package finished`, no `Failed` line.

- [ ] **Step 7: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: clamp MuJoCo PID position output with position_max_effort"
```

---

### Task 4: Independent render and camera decimation

The `headless`, `render_every_frame`, and `show_menus` parameters the course launch passes are read by nothing in this plugin version — the viewer redraws once per `1/60` simulated second and cameras publish once per `1/6`, both hard-coded. Under software rendering in the VM this dominates run time. This task turns both periods into parameters whose defaults reproduce today's behavior exactly, so validation runs can decimate the viewer without touching physics or sensor semantics.

**Files:**
- Modify (VM): `src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_node.cpp`

**Interfaces:**
- Consumes: the telemetry wiring from Task 2.
- Produces: ROS2 parameters `viewer_render_period` (double, simulated seconds between viewer redraws, default `0.0166666667`) and `camera_period` (double, simulated seconds between camera publishes, default `0.1666666667`). Task 11's launch file sets `viewer_render_period` high for batch validation.

- [ ] **Step 1: Read the parameters before the main loop**

In `src/mujoco_ros2_control_node.cpp`, replace:

```cpp
  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  mjtNum last_cam_update = mujoco_data->time;
```

with:

```cpp
  // Rendering and camera cadence are decimated independently of physics.
  // Defaults reproduce the original hard-coded 60 Hz viewer / 6 Hz cameras.
  double viewer_render_period = 1.0 / 60.0;
  double camera_period = 1.0 / 6.0;
  if (node->has_parameter("viewer_render_period"))
  {
    viewer_render_period = node->get_parameter("viewer_render_period").as_double();
  }
  if (node->has_parameter("camera_period"))
  {
    camera_period = node->get_parameter("camera_period").as_double();
  }
  RCLCPP_INFO(
    node->get_logger(), "Viewer render period %.4f s sim, camera period %.4f s sim",
    viewer_render_period, camera_period);

  mjtNum last_cam_update = mujoco_data->time;
```

- [ ] **Step 2: Use the parameters in the loop**

Replace:

```cpp
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0 / 60.0)
    {
      mujoco_control.update();
      telemetry->update(mujoco_model, mujoco_data);
    }
    rendering->update();

    // Updating cameras at ~6 Hz
    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > 1.0 / 6.0)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
```

with:

```cpp
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < viewer_render_period)
    {
      mujoco_control.update();
      telemetry->update(mujoco_model, mujoco_data);
    }
    rendering->update();

    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > camera_period)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
```

- [ ] **Step 3: Build**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && source install_repro/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select mujoco_ros2_control"
```
Expected: `Summary: 1 package finished`, no `Failed` line.

- [ ] **Step 4: Verify defaults are unchanged for the demo launch**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && timeout 40 ros2 launch openarm_moveit_config run_simulation_with_detector.launch.py 2>&1 | grep -m1 'Viewer render period'"
```
Expected: `Viewer render period 0.0167 s sim, camera period 0.1667 s sim`.

- [ ] **Step 5: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: make viewer render and camera periods configurable"
```

---
### Task 5: Python logic package skeleton and declared configuration

All grasp logic lives in a plain-Python package so it can be unit tested without ROS2 or MuJoCo. Tests run directly from the source directory: `cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test -q`.

**Files:**
- Create (VM): `src/openarm_grasp_physics/package.xml`
- Create (VM): `src/openarm_grasp_physics/setup.py`
- Create (VM): `src/openarm_grasp_physics/setup.cfg`
- Create (VM): `src/openarm_grasp_physics/resource/openarm_grasp_physics`
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/__init__.py`
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/config.py`
- Test (VM): `src/openarm_grasp_physics/test/test_config.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `GraspConfig` frozen dataclass with fields `object_geom`, `left_finger_geom`, `right_finger_geom`, `min_normal_force`, `confirm_window_s`, `lift_height_m`, `hold_duration_s`, `min_bilateral_ratio`, `max_drift_m`, `open_position`, `close_position`, `open_tolerance`, `hover_offset_m`, `grasp_z_offset_m`, `contact_confirm_timeout_s`, `gripper_action_timeout_s`, `plan_timeout_s`, `settle_duration_s`, and method `snapshot() -> dict`. Every later task imports `GraspConfig` from `openarm_grasp_physics.config`.

- [ ] **Step 1: Write the failing test**

Create `src/openarm_grasp_physics/test/test_config.py`:

```python
import pytest

from openarm_grasp_physics.config import GraspConfig


def test_defaults_match_the_declared_stage_1_thresholds():
    config = GraspConfig()

    assert config.lift_height_m == 0.020
    assert config.hold_duration_s == 2.0
    assert config.min_bilateral_ratio == 0.80
    assert config.max_drift_m == 0.010
    assert config.min_normal_force == 0.5
    assert config.confirm_window_s == 0.30


def test_geom_names_match_the_physics_model():
    config = GraspConfig()

    assert config.object_geom == "physics_object_geom"
    assert config.left_finger_geom == "openarm_left_left_finger_collision"
    assert config.right_finger_geom == "openarm_left_right_finger_collision"


def test_config_is_immutable():
    config = GraspConfig()

    with pytest.raises(Exception):
        config.lift_height_m = 0.5


def test_snapshot_is_a_plain_dict_for_trial_records():
    snapshot = GraspConfig().snapshot()

    assert isinstance(snapshot, dict)
    assert snapshot["lift_height_m"] == 0.020
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_config.py -q"
```
Expected: FAIL with `ModuleNotFoundError: No module named 'openarm_grasp_physics'`.

- [ ] **Step 3: Write the package skeleton and `config.py`**

`src/openarm_grasp_physics/openarm_grasp_physics/__init__.py` is empty.

`src/openarm_grasp_physics/openarm_grasp_physics/config.py`:

```python
"""Declared Stage 1 grasp configuration.

Every value here is a research parameter: changing one changes what
"successful grasp" means. They live in a single frozen dataclass so a
snapshot can be written into every trial record.
"""

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class GraspConfig:
    # MuJoCo geom names used for contact classification.
    object_geom: str = "physics_object_geom"
    left_finger_geom: str = "openarm_left_left_finger_collision"
    right_finger_geom: str = "openarm_left_right_finger_collision"

    # Contact acceptance.
    min_normal_force: float = 0.5
    confirm_window_s: float = 0.30

    # Grasp acceptance.
    lift_height_m: float = 0.020
    hold_duration_s: float = 2.0
    min_bilateral_ratio: float = 0.80
    max_drift_m: float = 0.010

    # Finger joint travel in metres. 0.0 is closed, 0.06 is fully open.
    open_position: float = 0.055
    close_position: float = 0.0
    open_tolerance: float = 0.005

    # Motion offsets in metres relative to the observed object surface.
    hover_offset_m: float = 0.08
    grasp_z_offset_m: float = 0.0

    # Timeouts, all in simulation seconds.
    settle_duration_s: float = 1.0
    contact_confirm_timeout_s: float = 5.0
    gripper_action_timeout_s: float = 10.0
    plan_timeout_s: float = 30.0

    def snapshot(self) -> dict:
        """Plain dict for embedding in trial records."""
        return asdict(self)
```

`src/openarm_grasp_physics/setup.py`:

```python
from setuptools import setup

package_name = "openarm_grasp_physics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nostalgia",
    maintainer_email="3096954761qq@gmail.com",
    description="Physics-mode grasp state machine and evaluation for OpenArm.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "physics_picker = openarm_grasp_physics.physics_picker:main",
        ],
    },
)
```

`src/openarm_grasp_physics/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/openarm_grasp_physics
[install]
install_scripts=$base/lib/openarm_grasp_physics
```

`src/openarm_grasp_physics/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>openarm_grasp_physics</name>
  <version>0.1.0</version>
  <description>Physics-mode grasp state machine and evaluation for OpenArm.</description>
  <maintainer email="3096954761qq@gmail.com">nostalgia</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>moveit_msgs</exec_depend>
  <exec_depend>shape_msgs</exec_depend>
  <exec_depend>control_msgs</exec_depend>
  <exec_depend>openarm_grasp_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`resource/openarm_grasp_physics` is an empty marker file:

```bash
ssh -p 2224 nostalgia@127.0.0.1 "mkdir -p ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/resource ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/test && touch ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/resource/openarm_grasp_physics"
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_config.py -q"
```
Expected: `4 passed`.

- [ ] **Step 5: Build the package so ROS2 can find it later**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && source install_repro/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select openarm_grasp_physics"
```
Expected: `Summary: 1 package finished`, no `Failed` line.

- [ ] **Step 6: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add openarm_grasp_physics package with declared grasp configuration"
```

---

### Task 6: Physics-mode MuJoCo model

Two problems in the baseline model are corrected here, and both are recorded as deliberate changes rather than silent edits:

1. The finger coupling `polycoef="0 -1 0 0 0"` means `q_finger1 = -q_finger2`. Both finger joints have range `0.0 0.06` and physically opposite slide axes, so the only value satisfying that constraint is zero. Equal coordinates (`polycoef="0 1 0 0 0"`) is the coupling the geometry actually implies.
2. The MJCF finger `position` actuators default to `ctrl = 0` (fully closed) with `kp="5000"`. Today they are overpowered by the plugin writing `qpos` directly every step. Once the finger is force-controlled, those actuators would slam the fingers shut, so they are removed from the physics model; the ROS2 PID force is then the only thing driving the left finger.

**Files:**
- Create (VM): `src/openarm_moveit_config/model/openarm_bimanual_physics.xml`
- Create (VM): `src/openarm_moveit_config/model/table_scene_physics.xml`
- Test (VM): `src/openarm_grasp_physics/test/test_physics_model_assets.py`

**Interfaces:**
- Consumes: `GraspConfig` from Task 5 for the expected geom name.
- Produces: a MuJoCo scene whose object is a single free rigid body named `physics_object` with geom `physics_object_geom`, no `cube_x/cube_y/cube_z` joints or actuators, equal-coordinate finger coupling, and no finger position actuators. Task 7's URDF and Task 12's launch file reference `table_scene_physics.xml`.

- [ ] **Step 1: Write the failing static test**

Create `src/openarm_grasp_physics/test/test_physics_model_assets.py`:

```python
"""Static guarantees about the physics-mode MuJoCo model.

These are the checks that stop a physical-grasp claim from silently
regressing into the software-attached demo.
"""
import os
import xml.etree.ElementTree as ET

import pytest

from openarm_grasp_physics.config import GraspConfig

MODEL_DIR = os.path.expanduser(
    "~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model"
)
SCENE = os.path.join(MODEL_DIR, "table_scene_physics.xml")
ROBOT = os.path.join(MODEL_DIR, "openarm_bimanual_physics.xml")


@pytest.fixture(scope="module")
def scene_root():
    return ET.parse(SCENE).getroot()


@pytest.fixture(scope="module")
def robot_root():
    return ET.parse(ROBOT).getroot()


def test_object_is_a_single_free_body(scene_root):
    bodies = [b for b in scene_root.iter("body") if b.get("name") == "physics_object"]
    assert len(bodies) == 1
    assert len(bodies[0].findall("freejoint")) == 1

    geoms = [g for g in bodies[0].findall("geom")]
    assert [g.get("name") for g in geoms] == [GraspConfig().object_geom]


def test_scene_has_no_object_slide_joints_or_actuators(scene_root):
    joint_names = {j.get("name") for j in scene_root.iter("joint")}
    assert joint_names.isdisjoint({"cube_x", "cube_y", "cube_z"})

    actuator_joints = {a.get("joint") for a in scene_root.iter("position")}
    assert actuator_joints.isdisjoint({"cube_x", "cube_y", "cube_z"})


def test_physics_timestep_is_unchanged(scene_root):
    options = list(scene_root.iter("option"))
    assert len(options) == 1
    assert options[0].get("timestep") == "0.002"


def test_finger_coupling_uses_equal_coordinates(robot_root):
    couplings = [
        j for j in robot_root.iter("joint")
        if j.get("joint1") is not None and "finger" in j.get("joint1")
    ]
    assert len(couplings) == 2
    for coupling in couplings:
        assert coupling.get("polycoef") == "0 1 0 0 0"


def test_finger_position_actuators_are_absent(robot_root):
    actuator_blocks = list(robot_root.iter("actuator"))
    assert len(actuator_blocks) == 1
    finger_actuators = [
        a for a in actuator_blocks[0]
        if a.get("joint") is not None and "finger" in a.get("joint")
    ]
    assert finger_actuators == []
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_physics_model_assets.py -q"
```
Expected: FAIL with `FileNotFoundError` for `table_scene_physics.xml`.

- [ ] **Step 3: Create the physics robot model**

Copy the baseline and apply exactly two changes:

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model && cp openarm_bimanual.xml openarm_bimanual_physics.xml && sed -i 's/polycoef=\"0 -1 0 0 0\"/polycoef=\"0 1 0 0 0\"/g' openarm_bimanual_physics.xml && sed -i '/<position name=\"left_finger[12]_ctrl\"/d; /<position name=\"right_finger[12]_ctrl\"/d' openarm_bimanual_physics.xml && grep -c 'polycoef=\"0 1 0 0 0\"' openarm_bimanual_physics.xml && grep -c 'finger.*_ctrl\" joint' openarm_bimanual_physics.xml"
```
Expected: `2` then `0`.

Note the `sed` deletion targets the two-line-per-actuator formatting; if `grep -c 'finger.*_ctrl\" joint'` is not `0`, remove the remaining `<position ... finger ... />` elements by hand before continuing.

- [ ] **Step 4: Create the physics scene**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model/table_scene_physics.xml" <<'EOF'
<mujoco model="openarm_bimanual_physics_scene">
  <include file="openarm_bimanual_physics.xml" actuator="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <statistic center="0 0 0.43" extent=".85" meansize="0.05"/>

  <visual>
    <headlight diffuse="0.5 0.5 0.5" ambient="0.4 0.4 0.4" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-10"/>
    <quality shadowsize="8192"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="champagne_wood" rgba="0.5 0.5 0.5 1" specular="0.2" shininess="0.3"/>
    <material name="white_carving" rgba="1.0 1.0 1.0 1" specular="0.1" shininess="0.1"/>
  </asset>

  <worldbody>
    <body name="cam_base" pos="0.5 0 1.2" euler="0 0 0">
      <camera name="my_cam" mode="fixed" fovy="45" resolution="640 480"/>
      <geom type="box" size="0.05 0.03 0.03" rgba="0.1 0.1 0.1 0" contype="0" conaffinity="0"/>
      <geom type="cylinder" size="0.01 0.05" pos="0 0 -0.05" rgba="1 0 0 0" contype="0" conaffinity="0"/>
    </body>

    <!-- physics_mode object: one free rigid body. Nothing may command its pose.
         Geometry, mass, friction and solver parameters are the untuned values
         carried over from the demo object so the first failure stays
         interpretable. -->
    <body name="physics_object" pos="0.3 0 0.46">
      <freejoint name="physics_object_free"/>
      <geom name="physics_object_geom"
            type="sphere"
            size="0.025"
            rgba="1 0 0 1"
            mass="0.1"
            condim="4"
            solref="0.005 1"
            solimp="0.95 0.99 0.001"
            margin="0.005"/>
    </body>

    <body name="debug_marker" pos="0 0 0">
        <joint name="marker_x" type="slide" axis="1 0 0" damping="0.5"/>
        <joint name="marker_y" type="slide" axis="0 1 0" damping="0.5"/>
        <joint name="marker_z" type="slide" axis="0 0 1" damping="0.5"/>
        <geom type="box" size="0.03 0.03 0.03" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
    </body>

    <body name="ornate_table" pos="0.5 0 0">
       <geom name="table_top" type="cylinder" size="0.35 0.02" pos="0 0 0.40" material="champagne_wood" contype="1" conaffinity="1" friction="0.05 0.01 0.01"/>
       <geom name="table_rim" type="cylinder" size="0.355 0.015" pos="0 0 0.385" material="white_carving"/>
       <geom name="base_cross_1" type="box" size="0.22 0.03 0.02" pos="0 0 0.06" material="champagne_wood"/>
       <geom name="base_cross_2" type="box" size="0.03 0.22 0.02" pos="0 0 0.06" material="champagne_wood"/>
       <geom name="center_knob" type="cylinder" size="0.04 0.05" pos="0 0 0.05" material="white_carving"/>
       <body name="leg_front">
         <geom type="capsule" fromto="0.25 0 0.06  0.20 0 0.22" size="0.025" material="champagne_wood"/>
         <geom type="capsule" fromto="0.20 0 0.22  0.30 0 0.38" size="0.035" material="champagne_wood"/>
         <geom type="sphere" pos="0.30 0 0.36" size="0.04" material="white_carving"/>
         <geom type="box" pos="0.25 0 0.04" size="0.04 0.03 0.02" material="champagne_wood"/>
       </body>
       <body name="leg_back">
         <geom type="capsule" fromto="-0.25 0 0.06  -0.20 0 0.22" size="0.025" material="champagne_wood"/>
         <geom type="capsule" fromto="-0.20 0 0.22  -0.30 0 0.38" size="0.035" material="champagne_wood"/>
         <geom type="sphere" pos="-0.30 0 0.36" size="0.04" material="white_carving"/>
         <geom type="box" pos="-0.25 0 0.04" size="0.04 0.03 0.02" material="champagne_wood"/>
       </body>
       <body name="leg_left">
         <geom type="capsule" fromto="0 0.25 0.06  0 0.20 0.22" size="0.025" material="champagne_wood"/>
         <geom type="capsule" fromto="0 0.20 0.22  0 0.30 0.38" size="0.035" material="champagne_wood"/>
         <geom type="sphere" pos="0 0.30 0.36" size="0.04" material="white_carving"/>
         <geom type="box" pos="0 0.25 0.04" size="0.03 0.04 0.02" material="champagne_wood"/>
       </body>
       <body name="leg_right">
         <geom type="capsule" fromto="0 -0.25 0.06  0 -0.20 0.22" size="0.025" material="champagne_wood"/>
         <geom type="capsule" fromto="0 -0.20 0.22  0 -0.30 0.38" size="0.035" material="champagne_wood"/>
         <geom type="sphere" pos="0 -0.30 0.36" size="0.04" material="white_carving"/>
         <geom type="box" pos="0 -0.25 0.04" size="0.03 0.04 0.02" material="champagne_wood"/>
       </body>
    </body>

    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>

  <actuator>
    <position joint="marker_x" kp="500" kv="50"/>
    <position joint="marker_y" kp="500" kv="50"/>
    <position joint="marker_z" kp="500" kv="50"/>
  </actuator>

</mujoco>
EOF
```

- [ ] **Step 5: Run the static test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_physics_model_assets.py -q"
```
Expected: `5 passed`.

- [ ] **Step 6: Verify MuJoCo compiles the model**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model && python3 -c \"import mujoco; m = mujoco.MjModel.from_xml_path('table_scene_physics.xml'); print('nq', m.nq, 'nu', m.nu); print('object geom id', mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'physics_object_geom'))\""
```
Expected: `nq` includes the 7 free-joint coordinates, and `object geom id` is a non-negative integer.

- [ ] **Step 7: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add physics-mode MuJoCo scene with free object and corrected finger coupling"
```

---

### Task 7: Physics-mode URDF, SRDF and controller configuration

Three consistency problems get fixed together, because MoveIt, `ros2_control` and MuJoCo must agree:

- the runtime URDF (`table_scene_full.urdf`) declares `cube_x/y/z` command interfaces that only exist to teleport the object;
- the SRDF declares `cube_group` and `marker_group` joints that appear in no URDF the MoveIt model loads, which is the source of the continuous `cube_x/y/z not found` errors;
- the left finger must switch from direct `qpos` writing to bounded PID force.

**Files:**
- Create (VM): `src/openarm_moveit_config/model/table_scene_physics.urdf`
- Create (VM): `src/openarm_moveit_config/config/openarm_physics.srdf`
- Create (VM): `src/openarm_moveit_config/launch/physics_controllers.yaml`
- Test (VM): `src/openarm_grasp_physics/test/test_physics_control_assets.py`

**Interfaces:**
- Consumes: the `position_max_effort` parameter from Task 3; the model from Task 6.
- Produces: a URDF with no `cube_*` joints, links, or interfaces, whose `openarm_left_finger_joint1` declares `<command_interface name="position_pid"/>` with `position_kp`, `position_kd`, `position_i_max`, `position_i_min` and `position_max_effort`; an SRDF with no `cube_group`, `marker_group`, or `red_cube_link` entries; a controller config with no `cube_controller`. Task 12's launch file loads all three.

- [ ] **Step 1: Write the failing static test**

Create `src/openarm_grasp_physics/test/test_physics_control_assets.py`:

```python
"""Static guarantees about the physics-mode URDF, SRDF and controller config."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.expanduser("~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config")
URDF = os.path.join(PKG, "model", "table_scene_physics.urdf")
SRDF = os.path.join(PKG, "config", "openarm_physics.srdf")
CONTROLLERS = os.path.join(PKG, "launch", "physics_controllers.yaml")

CUBE_JOINTS = {"cube_x", "cube_y", "cube_z"}


@pytest.fixture(scope="module")
def urdf_root():
    return ET.parse(URDF).getroot()


@pytest.fixture(scope="module")
def srdf_root():
    return ET.parse(SRDF).getroot()


@pytest.fixture(scope="module")
def controllers():
    with open(CONTROLLERS) as handle:
        return yaml.safe_load(handle)


def test_urdf_has_no_object_joints_links_or_interfaces(urdf_root):
    joint_names = {j.get("name") for j in urdf_root.iter("joint")}
    assert joint_names.isdisjoint(CUBE_JOINTS)

    link_names = {link.get("name") for link in urdf_root.iter("link")}
    assert link_names.isdisjoint({"cube_link_1", "cube_link_2", "red_cube_link"})


def test_left_finger_uses_bounded_pid_force(urdf_root):
    control = urdf_root.find("ros2_control")
    finger = [
        j for j in control.findall("joint")
        if j.get("name") == "openarm_left_finger_joint1"
    ]
    assert len(finger) == 1

    interfaces = {c.get("name") for c in finger[0].findall("command_interface")}
    assert interfaces == {"position_pid"}

    params = {p.get("name"): p.text for p in finger[0].iter("param")}
    assert "position_kp" in params
    assert float(params["position_max_effort"]) == 20.0


def test_arm_joints_keep_the_plain_position_interface(urdf_root):
    control = urdf_root.find("ros2_control")
    arm = [
        j for j in control.findall("joint")
        if j.get("name") == "openarm_left_joint1"
    ]
    interfaces = {c.get("name") for c in arm[0].findall("command_interface")}
    assert interfaces == {"position"}


def test_srdf_declares_no_joint_absent_from_the_moveit_model(srdf_root):
    joint_names = {j.get("name") for j in srdf_root.iter("joint")}
    assert joint_names.isdisjoint(CUBE_JOINTS)
    assert joint_names.isdisjoint({"marker_x", "marker_y", "marker_z"})

    group_names = {g.get("name") for g in srdf_root.iter("group")}
    assert "cube_group" not in group_names
    assert "marker_group" not in group_names

    collision_links = set()
    for entry in srdf_root.iter("disable_collisions"):
        collision_links.add(entry.get("link1"))
        collision_links.add(entry.get("link2"))
    assert "red_cube_link" not in collision_links


def test_controller_config_has_no_object_controller(controllers):
    manager = controllers["controller_manager"]["ros__parameters"]
    assert "cube_controller" not in manager
    assert "cube_controller" not in controllers
    assert manager["update_rate"] == 100
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_physics_control_assets.py -q"
```
Expected: FAIL with `FileNotFoundError` for `table_scene_physics.urdf`.

- [ ] **Step 3: Create the physics URDF**

Start from the baseline, then delete the object chain and rewrite the left finger interface:

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model && cp table_scene_full.urdf table_scene_physics.urdf"
```

Then apply these edits to `table_scene_physics.urdf`:

Edit A — delete the three object links and three object joints (`<link name="cube_link_1"/>` through the end of `<joint name="cube_z" ...>`).

Edit B — delete the three object entries from the `ros2_control` block:

```xml
    <joint name="cube_x">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="cube_y">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="cube_z">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
```

Edit C — replace the left finger's `ros2_control` entry with the bounded PID form. The `position_pid` interface name is what switches the plugin from writing `qpos` to writing a clamped `qfrc_applied`; the plugin still exposes it to controllers as a plain `position` interface, so `left_hand_controller` needs no change:

```xml
    <joint name="openarm_left_finger_joint1">
      <command_interface name="position_pid"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="position_kp">2000.0</param>
      <param name="position_ki">0.0</param>
      <param name="position_kd">20.0</param>
      <param name="position_i_max">5.0</param>
      <param name="position_i_min">-5.0</param>
      <param name="position_max_effort">20.0</param>
    </joint>
```

Edit D — reduce the finger effort limit in the kinematic description from `333` to `20`, matching the clamp, for both left finger joints:

```xml
    <limit lower="0.0" upper="0.06" effort="20" velocity="0.2"/>
```

- [ ] **Step 4: Create the physics SRDF**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/config && cp openarm.srdf openarm_physics.srdf"
```

Then delete from `openarm_physics.srdf`:
- the `<group name="marker_group">` block and the `<group name="cube_group">` block;
- both `<group_state name="zero" ...>` blocks that reference those groups;
- every `<disable_collisions ...>` line mentioning `red_cube_link`.

Verify:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "grep -c 'cube\|marker' ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/config/openarm_physics.srdf"
```
Expected: `0`.

- [ ] **Step 5: Create the physics controller configuration**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/launch/physics_controllers.yaml" <<'EOF'
# physics_mode controllers. No object controller exists in this mode: the
# object is a MuJoCo free body and nothing is permitted to command its pose.
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    left_hand_controller:
      type: position_controllers/GripperActionController

    right_hand_controller:
      type: position_controllers/GripperActionController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

left_arm_controller:
  ros__parameters:
    joints:
      - openarm_left_joint1
      - openarm_left_joint2
      - openarm_left_joint3
      - openarm_left_joint4
      - openarm_left_joint5
      - openarm_left_joint6
      - openarm_left_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
right_arm_controller:
  ros__parameters:
    joints:
      - openarm_right_joint1
      - openarm_right_joint2
      - openarm_right_joint3
      - openarm_right_joint4
      - openarm_right_joint5
      - openarm_right_joint6
      - openarm_right_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
left_hand_controller:
  ros__parameters:
    joint: openarm_left_finger_joint1
    max_effort: 20.0
    allow_stalling: true
    stall_velocity_threshold: 0.001
    stall_timeout: 0.5
right_hand_controller:
  ros__parameters:
    joint: openarm_right_finger_joint1
EOF
```

- [ ] **Step 6: Run the static test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_physics_control_assets.py -q"
```
Expected: `5 passed`.

- [ ] **Step 7: Verify the URDF parses**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && python3 -c \"import xml.etree.ElementTree as ET; ET.parse('/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model/table_scene_physics.urdf'); print('urdf ok')\" && check_urdf ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model/table_scene_physics.urdf | head -3"
```
Expected: `urdf ok` followed by a successful `robot name is:` line. If `check_urdf` is unavailable, the Python parse alone is sufficient.

- [ ] **Step 8: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add physics-mode URDF, SRDF and controller configuration"
```

---

### Task 8: Contact classification

**Files:**
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/contact.py`
- Test (VM): `src/openarm_grasp_physics/test/test_contact.py`

**Interfaces:**
- Consumes: `GraspConfig` from Task 5; the `contact_geoms` / `contact_normal_forces` arrays from the Task 1 message.
- Produces: `ContactSummary` frozen dataclass with fields `left_contact: bool`, `right_contact: bool`, `bilateral: bool`, `left_normal_force: float`, `right_normal_force: float`, and the function `summarize_contacts(geoms: Sequence[str], normal_forces: Sequence[float], config: GraspConfig) -> ContactSummary`.

- [ ] **Step 1: Write the failing test**

Create `src/openarm_grasp_physics/test/test_contact.py`:

```python
import pytest

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.contact import ContactSummary, summarize_contacts

CONFIG = GraspConfig()
LEFT = CONFIG.left_finger_geom
RIGHT = CONFIG.right_finger_geom


def test_no_contacts_is_no_grasp():
    summary = summarize_contacts([], [], CONFIG)

    assert summary == ContactSummary(False, False, False, 0.0, 0.0)


def test_one_finger_is_unilateral():
    summary = summarize_contacts([LEFT], [3.0], CONFIG)

    assert summary.left_contact is True
    assert summary.right_contact is False
    assert summary.bilateral is False


def test_both_fingers_is_bilateral():
    summary = summarize_contacts([LEFT, RIGHT], [3.0, 2.0], CONFIG)

    assert summary.bilateral is True
    assert summary.left_normal_force == 3.0
    assert summary.right_normal_force == 2.0


def test_table_and_palm_contacts_are_ignored():
    summary = summarize_contacts(
        ["table_top", "openarm_left_hand_collision", LEFT],
        [12.0, 8.0, 3.0],
        CONFIG,
    )

    assert summary.left_contact is True
    assert summary.right_contact is False
    assert summary.right_normal_force == 0.0


def test_multiple_contacts_on_one_finger_are_summed():
    summary = summarize_contacts([LEFT, LEFT, RIGHT], [1.0, 2.5, 4.0], CONFIG)

    assert summary.left_normal_force == 3.5
    assert summary.bilateral is True


def test_force_below_threshold_is_not_contact():
    summary = summarize_contacts([LEFT, RIGHT], [0.4, 4.0], CONFIG)

    assert summary.left_contact is False
    assert summary.bilateral is False
    assert summary.left_normal_force == 0.4


def test_force_exactly_at_threshold_counts():
    summary = summarize_contacts([LEFT, RIGHT], [0.5, 0.5], CONFIG)

    assert summary.bilateral is True


def test_negative_normal_force_is_treated_as_magnitude():
    summary = summarize_contacts([LEFT, RIGHT], [-3.0, 3.0], CONFIG)

    assert summary.left_normal_force == 3.0
    assert summary.bilateral is True


def test_mismatched_array_lengths_are_rejected():
    with pytest.raises(ValueError):
        summarize_contacts([LEFT, RIGHT], [1.0], CONFIG)
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_contact.py -q"
```
Expected: FAIL with `ModuleNotFoundError: No module named 'openarm_grasp_physics.contact'`.

- [ ] **Step 3: Write the implementation**

Create `src/openarm_grasp_physics/openarm_grasp_physics/contact.py`:

```python
"""Classification of raw MuJoCo object contacts into a grasp state.

The simulator publishes every contact the object is involved in. Only the two
named left-hand finger geoms count towards a grasp: palm, arm and table
contacts are evidence of collision, not of grasping.
"""

from dataclasses import dataclass
from typing import Sequence

from openarm_grasp_physics.config import GraspConfig


@dataclass(frozen=True)
class ContactSummary:
    left_contact: bool
    right_contact: bool
    bilateral: bool
    left_normal_force: float
    right_normal_force: float


def summarize_contacts(
    geoms: Sequence[str],
    normal_forces: Sequence[float],
    config: GraspConfig,
) -> ContactSummary:
    if len(geoms) != len(normal_forces):
        raise ValueError(
            "geoms and normal_forces must have equal length, got "
            f"{len(geoms)} and {len(normal_forces)}"
        )

    left_force = 0.0
    right_force = 0.0
    for name, force in zip(geoms, normal_forces):
        magnitude = abs(float(force))
        if name == config.left_finger_geom:
            left_force += magnitude
        elif name == config.right_finger_geom:
            right_force += magnitude

    left_contact = left_force >= config.min_normal_force
    right_contact = right_force >= config.min_normal_force

    return ContactSummary(
        left_contact=left_contact,
        right_contact=right_contact,
        bilateral=left_contact and right_contact,
        left_normal_force=left_force,
        right_normal_force=right_force,
    )
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_contact.py -q"
```
Expected: `9 passed`.

- [ ] **Step 5: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: classify object contacts into bilateral grasp state"
```

---

### Task 9: Relative pose, elevation and slip geometry

**Files:**
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/geometry.py`
- Test (VM): `src/openarm_grasp_physics/test/test_geometry.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `relative_position(object_xyz, gripper_xyz, gripper_quat_wxyz) -> tuple[float, float, float]` (object position expressed in the gripper frame), `distance(a, b) -> float`, and `elevation(object_z, baseline_z) -> float`.

- [ ] **Step 1: Write the failing test**

Create `src/openarm_grasp_physics/test/test_geometry.py`:

```python
import math

from openarm_grasp_physics.geometry import distance, elevation, relative_position

IDENTITY = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))


def assert_vec_close(actual, expected, abs_tol=1e-9):
    for a, e in zip(actual, expected):
        assert math.isclose(a, e, abs_tol=abs_tol), f"{actual} != {expected}"


def test_identity_rotation_is_a_plain_offset():
    result = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)

    # 0.5 - 0.6 is not exactly representable in binary64 (it lands near
    # -0.09999999999999998, not the literal -0.1), so this compares with a
    # tolerance rather than "==" — the geometry, not raw bit patterns, is
    # what this test locks in.
    assert_vec_close(result, (0.0, 0.0, -0.1))


def test_gripper_rotation_is_applied():
    result = relative_position((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), YAW_90)

    # sqrt(0.5) is irrational; its rotation-matrix terms carry a residual on
    # the order of 1e-16 even for the "should be exactly zero" components, so
    # this compares with a tolerance rather than "==".
    assert_vec_close(result, (0.0, -1.0, 0.0))


def test_distance_is_euclidean():
    assert distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == 5.0


def test_zero_drift_when_relative_pose_is_unchanged():
    grasp = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)
    lifted = relative_position((0.3, 0.0, 0.55), (0.3, 0.0, 0.65), IDENTITY)

    assert math.isclose(distance(grasp, lifted), 0.0, abs_tol=1e-9)


def test_slip_shows_up_as_relative_drift():
    grasp = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)
    slipped = relative_position((0.3, 0.0, 0.49), (0.3, 0.0, 0.65), IDENTITY)

    assert math.isclose(distance(grasp, slipped), 0.06, rel_tol=1e-9)


def test_elevation_is_measured_from_the_settled_baseline():
    assert math.isclose(elevation(0.465, 0.445), 0.020, rel_tol=1e-9)
```

Note on the `test_gripper_rotation_is_applied` expectation: rotating the gripper frame by `+90°` about `z` puts a point at world `+x` on the gripper frame's `-y` axis.

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_geometry.py -q"
```
Expected: FAIL with `ModuleNotFoundError: No module named 'openarm_grasp_physics.geometry'`.

- [ ] **Step 3: Write the implementation**

Create `src/openarm_grasp_physics/openarm_grasp_physics/geometry.py`:

```python
"""Pose arithmetic for grasp evidence.

Slip is only meaningful in the gripper frame: an object carried perfectly by a
moving gripper has large world motion and zero relative motion.
"""

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]


def _rotation_matrix(quat_wxyz: Sequence[float]):
    w, x, y, z = (float(v) for v in quat_wxyz)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def relative_position(
    object_xyz: Sequence[float],
    gripper_xyz: Sequence[float],
    gripper_quat_wxyz: Sequence[float],
) -> Vec3:
    """Object position expressed in the gripper frame."""
    delta = tuple(float(o) - float(g) for o, g in zip(object_xyz, gripper_xyz))
    rotation = _rotation_matrix(gripper_quat_wxyz)
    # Transpose of the rotation maps world vectors into the gripper frame.
    return tuple(
        sum(rotation[row][col] * delta[row] for row in range(3)) for col in range(3)
    )


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(p) - float(q)) ** 2 for p, q in zip(a, b)))


def elevation(object_z: float, baseline_z: float) -> float:
    return float(object_z) - float(baseline_z)
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_geometry.py -q"
```
Expected: `6 passed`.

- [ ] **Step 5: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add relative pose, elevation and slip geometry"
```

---

### Task 10: Failure classification and success gate

**Files:**
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/evaluation.py`
- Test (VM): `src/openarm_grasp_physics/test/test_evaluation.py`

**Interfaces:**
- Consumes: `GraspConfig` from Task 5.
- Produces: `FailureCategory` string enum with members `PLANNING_FAILURE`, `GRIPPER_COMMAND_FAILURE`, `NO_CONTACT`, `UNILATERAL_CONTACT`, `CONTACT_LOST_DURING_LIFT`, `INSUFFICIENT_LIFT`, `EXCESSIVE_SLIP`, `HOLD_FAILURE`, `SUCCESS`; `TrialObservations` frozen dataclass with fields `planning_ok`, `gripper_ok`, `timed_out`, `any_contact`, `bilateral_confirmed`, `contact_lost_during_lift`, `max_elevation_m`, `hold_completed`, `hold_bilateral_ratio`, `max_drift_m`; and `classify_trial(observations, config) -> FailureCategory`.

- [ ] **Step 1: Write the failing test**

Create `src/openarm_grasp_physics/test/test_evaluation.py`:

```python
import dataclasses

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import (
    FailureCategory,
    TrialObservations,
    classify_trial,
)

CONFIG = GraspConfig()

PASSING = TrialObservations(
    planning_ok=True,
    gripper_ok=True,
    timed_out=False,
    any_contact=True,
    bilateral_confirmed=True,
    contact_lost_during_lift=False,
    max_elevation_m=0.025,
    hold_completed=True,
    hold_bilateral_ratio=0.95,
    max_drift_m=0.004,
)


def observations(**overrides):
    return dataclasses.replace(PASSING, **overrides)


def test_all_conditions_met_is_success():
    assert classify_trial(PASSING, CONFIG) is FailureCategory.SUCCESS


def test_planning_failure_wins_over_everything():
    result = classify_trial(
        observations(planning_ok=False, any_contact=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.PLANNING_FAILURE


def test_gripper_command_failure_precedes_contact_categories():
    result = classify_trial(observations(gripper_ok=False, any_contact=False), CONFIG)

    assert result is FailureCategory.GRIPPER_COMMAND_FAILURE


def test_no_contact_at_all():
    result = classify_trial(
        observations(any_contact=False, bilateral_confirmed=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.NO_CONTACT


def test_contact_that_never_became_bilateral():
    result = classify_trial(
        observations(bilateral_confirmed=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.UNILATERAL_CONTACT


def test_contact_lost_during_lift():
    result = classify_trial(
        observations(contact_lost_during_lift=True, max_elevation_m=0.005),
        CONFIG,
    )

    assert result is FailureCategory.CONTACT_LOST_DURING_LIFT


def test_lift_below_threshold():
    result = classify_trial(observations(max_elevation_m=0.019), CONFIG)

    assert result is FailureCategory.INSUFFICIENT_LIFT


def test_lift_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(max_elevation_m=0.020), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_drift_above_threshold():
    result = classify_trial(observations(max_drift_m=0.0101), CONFIG)

    assert result is FailureCategory.EXCESSIVE_SLIP


def test_drift_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(max_drift_m=0.010), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_hold_not_completed():
    result = classify_trial(observations(hold_completed=False), CONFIG)

    assert result is FailureCategory.HOLD_FAILURE


def test_hold_bilateral_ratio_below_threshold():
    result = classify_trial(observations(hold_bilateral_ratio=0.79), CONFIG)

    assert result is FailureCategory.HOLD_FAILURE


def test_hold_bilateral_ratio_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(hold_bilateral_ratio=0.80), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_timeout_is_reported_as_planning_failure_when_planning_failed():
    result = classify_trial(observations(planning_ok=False, timed_out=True), CONFIG)

    assert result is FailureCategory.PLANNING_FAILURE


def test_monitoring_timeout_alone_blocks_success():
    result = classify_trial(observations(timed_out=True), CONFIG)

    assert result is not FailureCategory.SUCCESS
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_evaluation.py -q"
```
Expected: FAIL with `ModuleNotFoundError: No module named 'openarm_grasp_physics.evaluation'`.

- [ ] **Step 3: Write the implementation**

Create `src/openarm_grasp_physics/openarm_grasp_physics/evaluation.py`:

```python
"""Deterministic single-category outcome for one grasp trial.

A trial may exhibit several symptoms at once. The canonical result is the
first category in the fixed precedence order below, so results stay
comparable across runs.
"""

from dataclasses import dataclass
from enum import Enum

from openarm_grasp_physics.config import GraspConfig


class FailureCategory(str, Enum):
    PLANNING_FAILURE = "planning_failure"
    GRIPPER_COMMAND_FAILURE = "gripper_command_failure"
    NO_CONTACT = "no_contact"
    UNILATERAL_CONTACT = "unilateral_contact"
    CONTACT_LOST_DURING_LIFT = "contact_lost_during_lift"
    INSUFFICIENT_LIFT = "insufficient_lift"
    EXCESSIVE_SLIP = "excessive_slip"
    HOLD_FAILURE = "hold_failure"
    SUCCESS = "success"


@dataclass(frozen=True)
class TrialObservations:
    planning_ok: bool
    gripper_ok: bool
    timed_out: bool
    any_contact: bool
    bilateral_confirmed: bool
    contact_lost_during_lift: bool
    max_elevation_m: float
    hold_completed: bool
    hold_bilateral_ratio: float
    max_drift_m: float


def classify_trial(
    observations: TrialObservations, config: GraspConfig
) -> FailureCategory:
    if not observations.planning_ok:
        return FailureCategory.PLANNING_FAILURE
    if not observations.gripper_ok:
        return FailureCategory.GRIPPER_COMMAND_FAILURE
    if not observations.any_contact:
        return FailureCategory.NO_CONTACT
    if not observations.bilateral_confirmed:
        return FailureCategory.UNILATERAL_CONTACT
    if observations.contact_lost_during_lift:
        return FailureCategory.CONTACT_LOST_DURING_LIFT
    if observations.max_elevation_m < config.lift_height_m:
        return FailureCategory.INSUFFICIENT_LIFT
    if observations.max_drift_m > config.max_drift_m:
        return FailureCategory.EXCESSIVE_SLIP
    if not observations.hold_completed:
        return FailureCategory.HOLD_FAILURE
    if observations.hold_bilateral_ratio < config.min_bilateral_ratio:
        return FailureCategory.HOLD_FAILURE
    if observations.timed_out:
        return FailureCategory.HOLD_FAILURE
    return FailureCategory.SUCCESS
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_evaluation.py -q"
```
Expected: `15 passed`.

- [ ] **Step 5: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add deterministic grasp failure classification"
```

---

### Task 11: Trial recording and reporting

**Files:**
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/trial_record.py`
- Test (VM): `src/openarm_grasp_physics/test/test_trial_record.py`

**Interfaces:**
- Consumes: `GraspConfig` (Task 5), `FailureCategory` and `TrialObservations` (Task 10).
- Produces: `Sample` frozen dataclass with fields `sim_time`, `stage`, `left_contact`, `right_contact`, `left_force`, `right_force`, `object_x/y/z`, `gripper_x/y/z`, `elevation`, `drift`; and `TrialRecorder` with `__init__(output_dir, trial_id, config)`, `add_sample(sample)`, `mark_stage(stage, sim_time)`, `finish(observations, category) -> dict`. `finish` writes `samples.csv`, `trial.json` and `report.md` under `output_dir/trial_id/` and returns the summary dict.

- [ ] **Step 1: Write the failing test**

Create `src/openarm_grasp_physics/test/test_trial_record.py`:

```python
import csv
import json
import os

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import FailureCategory, TrialObservations
from openarm_grasp_physics.trial_record import Sample, TrialRecorder

CONFIG = GraspConfig()

OBSERVATIONS = TrialObservations(
    planning_ok=True,
    gripper_ok=True,
    timed_out=False,
    any_contact=True,
    bilateral_confirmed=True,
    contact_lost_during_lift=False,
    max_elevation_m=0.024,
    hold_completed=True,
    hold_bilateral_ratio=0.93,
    max_drift_m=0.003,
)


def make_sample(sim_time, stage="hold"):
    return Sample(
        sim_time=sim_time,
        stage=stage,
        left_contact=True,
        right_contact=True,
        left_force=4.0,
        right_force=3.5,
        object_x=0.30,
        object_y=0.0,
        object_z=0.469,
        gripper_x=0.30,
        gripper_y=0.0,
        gripper_z=0.55,
        elevation=0.024,
        drift=0.003,
    )


def test_finish_writes_all_three_products(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_001", CONFIG)
    recorder.mark_stage("close", 1.0)
    recorder.add_sample(make_sample(1.02))
    recorder.add_sample(make_sample(1.04))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    trial_dir = os.path.join(str(tmp_path), "trial_001")
    assert sorted(os.listdir(trial_dir)) == ["report.md", "samples.csv", "trial.json"]
    assert summary["success"] is True
    assert summary["category"] == "success"


def test_samples_csv_has_one_row_per_sample(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_002", CONFIG)
    recorder.add_sample(make_sample(0.5))
    recorder.add_sample(make_sample(0.6))
    recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    with open(os.path.join(str(tmp_path), "trial_002", "samples.csv")) as handle:
        rows = list(csv.DictReader(handle))

    assert len(rows) == 2
    assert rows[0]["sim_time"] == "0.5"
    assert rows[1]["stage"] == "hold"


def test_trial_json_embeds_the_config_snapshot_and_stage_times(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_003", CONFIG)
    recorder.mark_stage("lift", 3.25)
    recorder.add_sample(make_sample(3.3))
    recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    with open(os.path.join(str(tmp_path), "trial_003", "trial.json")) as handle:
        payload = json.load(handle)

    assert payload["config"]["lift_height_m"] == 0.020
    assert payload["stage_times"]["lift"] == 3.25
    assert payload["observations"]["max_elevation_m"] == 0.024
    assert payload["sample_count"] == 1


def test_failure_is_recorded_as_not_successful(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_004", CONFIG)
    recorder.add_sample(make_sample(0.1))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.EXCESSIVE_SLIP)

    assert summary["success"] is False
    assert summary["category"] == "excessive_slip"


def test_report_markdown_names_the_category(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_005", CONFIG)
    recorder.add_sample(make_sample(0.1))
    recorder.finish(OBSERVATIONS, FailureCategory.INSUFFICIENT_LIFT)

    with open(os.path.join(str(tmp_path), "trial_005", "report.md")) as handle:
        report = handle.read()

    assert "insufficient_lift" in report
    assert "trial_005" in report
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_trial_record.py -q"
```
Expected: FAIL with `ModuleNotFoundError: No module named 'openarm_grasp_physics.trial_record'`.

- [ ] **Step 3: Write the implementation**

Create `src/openarm_grasp_physics/openarm_grasp_physics/trial_record.py`:

```python
"""Per-trial data products: raw samples, machine summary, human summary."""

import csv
import dataclasses
import json
import os
from dataclasses import dataclass
from typing import Dict, List

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import FailureCategory, TrialObservations


@dataclass(frozen=True)
class Sample:
    sim_time: float
    stage: str
    left_contact: bool
    right_contact: bool
    left_force: float
    right_force: float
    object_x: float
    object_y: float
    object_z: float
    gripper_x: float
    gripper_y: float
    gripper_z: float
    elevation: float
    drift: float


class TrialRecorder:
    def __init__(self, output_dir: str, trial_id: str, config: GraspConfig):
        self._trial_dir = os.path.join(output_dir, trial_id)
        self._trial_id = trial_id
        self._config = config
        self._samples: List[Sample] = []
        self._stage_times: Dict[str, float] = {}

    def add_sample(self, sample: Sample) -> None:
        self._samples.append(sample)

    def mark_stage(self, stage: str, sim_time: float) -> None:
        self._stage_times[stage] = float(sim_time)

    def finish(
        self, observations: TrialObservations, category: FailureCategory
    ) -> dict:
        os.makedirs(self._trial_dir, exist_ok=True)
        self._write_samples()

        summary = {
            "trial_id": self._trial_id,
            "mode": "physics_mode",
            "category": category.value,
            "success": category is FailureCategory.SUCCESS,
            "sample_count": len(self._samples),
            "stage_times": self._stage_times,
            "observations": dataclasses.asdict(observations),
            "config": self._config.snapshot(),
        }

        with open(os.path.join(self._trial_dir, "trial.json"), "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)

        self._write_report(summary)
        return summary

    def _write_samples(self) -> None:
        fields = [field.name for field in dataclasses.fields(Sample)]
        path = os.path.join(self._trial_dir, "samples.csv")
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for sample in self._samples:
                writer.writerow(dataclasses.asdict(sample))

    def _write_report(self, summary: dict) -> None:
        observations = summary["observations"]
        lines = [
            "# Physics-mode grasp trial {}".format(self._trial_id),
            "",
            "- Result: `{}`".format(summary["category"]),
            "- Success: `{}`".format(summary["success"]),
            "- Samples: `{}`".format(summary["sample_count"]),
            "- Max elevation: `{:.4f} m` (threshold `{:.3f} m`)".format(
                observations["max_elevation_m"], self._config.lift_height_m
            ),
            "- Hold bilateral ratio: `{:.3f}` (threshold `{:.2f}`)".format(
                observations["hold_bilateral_ratio"], self._config.min_bilateral_ratio
            ),
            "- Max relative drift: `{:.4f} m` (threshold `{:.3f} m`)".format(
                observations["max_drift_m"], self._config.max_drift_m
            ),
            "",
            "## Stage times (simulation seconds)",
            "",
        ]
        for stage, sim_time in sorted(
            summary["stage_times"].items(), key=lambda item: item[1]
        ):
            lines.append("- `{}`: `{:.3f}`".format(stage, sim_time))
        lines.append("")
        lines.append(
            "No software attachment and no object pose command exist in this mode; "
            "object motion is produced only by contact."
        )
        lines.append("")

        with open(os.path.join(self._trial_dir, "report.md"), "w") as handle:
            handle.write("\n".join(lines))
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test/test_trial_record.py -q"
```
Expected: `5 passed`.

- [ ] **Step 5: Run the whole suite**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test -q"
```
Expected: `44 passed`.

- [ ] **Step 6: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: record grasp trial samples, summary and report"
```

---

### Task 12: Physics-mode grasp node and launch files

**Files:**
- Create (VM): `src/openarm_grasp_physics/openarm_grasp_physics/physics_picker.py`
- Create (VM): `src/openarm_moveit_config/launch/move_group_physics.launch.py`
- Create (VM): `src/openarm_moveit_config/launch/run_physics_grasp.launch.py`

**Interfaces:**
- Consumes: everything from Tasks 1 and 5 through 11.
- Produces: executable `ros2 run openarm_grasp_physics physics_picker --ros-args -p use_sim_time:=true -p trial_id:=<id> -p output_dir:=<dir>`, which runs exactly one bounded trial and exits with code `0` on success and `1` on any failure category; launch file `run_physics_grasp.launch.py` bringing up MuJoCo in physics mode, the detector, MoveIt, and the table planning-scene object, without any picker.

- [ ] **Step 1: Write the grasp node**

Create `src/openarm_grasp_physics/openarm_grasp_physics/physics_picker.py`:

```python
#!/usr/bin/env python3
"""physics_mode single-trial grasp state machine.

There is no software attachment and no object pose publisher here. The object
is a MuJoCo free body; if it moves with the gripper, contact moved it.

All waits are measured in simulation time taken from the telemetry stream,
because the VM real-time factor is well below 1.0 and wall-clock sleeps are
therefore not evidence windows.
"""

import math
import sys
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from openarm_grasp_msgs.msg import GraspRawState
from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.contact import summarize_contacts
from openarm_grasp_physics.evaluation import (
    FailureCategory,
    TrialObservations,
    classify_trial,
)
from openarm_grasp_physics.geometry import distance, elevation, relative_position
from openarm_grasp_physics.trial_record import Sample, TrialRecorder

GROUP = "left_arm"
EE_LINK = "openarm_left_hand_tcp"
GRIPPER_ACTION = "/left_hand_controller/gripper_cmd"
PROTECTION_ID = "grasp_protection_sphere"
PROTECTION_RADIUS = 0.06


class PhysicsPicker(Node):
    def __init__(self):
        super().__init__("physics_picker")

        self.declare_parameter("trial_id", "trial_000")
        self.declare_parameter(
            "output_dir", "/home/nostalgia/ros2_ws_course_repro/results/physics_grasp"
        )

        self.config = GraspConfig()
        self.trial_id = self.get_parameter("trial_id").value
        self.output_dir = self.get_parameter("output_dir").value
        self.recorder = TrialRecorder(self.output_dir, self.trial_id, self.config)

        self._lock = threading.Lock()
        self._latest = None            # last GraspRawState
        self._stage = "startup"
        self._baseline_z = None
        self._grasp_relative = None

        self._max_elevation = 0.0
        self._max_drift = 0.0
        self._any_contact = False
        self._bilateral_confirmed = False
        self._contact_lost_during_lift = False
        self._hold_total = 0
        self._hold_bilateral = 0

        self._target = None
        self.create_subscription(Point, "/apple_location", self._on_target, 10)
        self.create_subscription(
            GraspRawState, "/grasp_raw_state", self._on_telemetry, 50
        )
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

    # ---------------- callbacks ----------------

    def _on_target(self, msg):
        if abs(msg.x) < 0.01 and abs(msg.y) < 0.01:
            return
        with self._lock:
            if self._target is None:
                self._target = (msg.x, msg.y, msg.z)

    def _on_telemetry(self, msg):
        summary = summarize_contacts(
            msg.contact_geoms, msg.contact_normal_forces, self.config
        )
        object_xyz = (
            msg.object_pose.position.x,
            msg.object_pose.position.y,
            msg.object_pose.position.z,
        )
        gripper_xyz = (
            msg.gripper_pose.position.x,
            msg.gripper_pose.position.y,
            msg.gripper_pose.position.z,
        )
        gripper_quat = (
            msg.gripper_pose.orientation.w,
            msg.gripper_pose.orientation.x,
            msg.gripper_pose.orientation.y,
            msg.gripper_pose.orientation.z,
        )
        relative = relative_position(object_xyz, gripper_xyz, gripper_quat)

        with self._lock:
            self._latest = msg
            stage = self._stage

            height = (
                elevation(object_xyz[2], self._baseline_z)
                if self._baseline_z is not None
                else 0.0
            )
            drift = (
                distance(relative, self._grasp_relative)
                if self._grasp_relative is not None
                else 0.0
            )
            self._relative = relative

            if summary.left_contact or summary.right_contact:
                self._any_contact = True
            if height > self._max_elevation:
                self._max_elevation = height
            if self._grasp_relative is not None and drift > self._max_drift:
                self._max_drift = drift
            if stage == "lift" and self._bilateral_confirmed and not summary.bilateral:
                self._contact_lost_during_lift = True
            if stage == "hold":
                self._hold_total += 1
                if summary.bilateral:
                    self._hold_bilateral += 1

            self._summary = summary
            self.recorder.add_sample(
                Sample(
                    sim_time=msg.sim_time,
                    stage=stage,
                    left_contact=summary.left_contact,
                    right_contact=summary.right_contact,
                    left_force=summary.left_normal_force,
                    right_force=summary.right_normal_force,
                    object_x=object_xyz[0],
                    object_y=object_xyz[1],
                    object_z=object_xyz[2],
                    gripper_x=gripper_xyz[0],
                    gripper_y=gripper_xyz[1],
                    gripper_z=gripper_xyz[2],
                    elevation=height,
                    drift=drift,
                )
            )

    # ---------------- simulation-time helpers ----------------

    def sim_time(self):
        with self._lock:
            return self._latest.sim_time if self._latest is not None else None

    def wait_for_telemetry(self, wall_timeout=60.0):
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            if self.sim_time() is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_sim(self, duration_s, wall_timeout=300.0):
        start = self.sim_time()
        if start is None:
            return False
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            now = self.sim_time()
            if now is not None and now - start >= duration_s:
                return True
            time.sleep(0.01)
        return False

    def set_stage(self, stage):
        with self._lock:
            self._stage = stage
        now = self.sim_time()
        if now is not None:
            self.recorder.mark_stage(stage, now)
        self.get_logger().info("stage -> {}".format(stage))

    # ---------------- motion primitives ----------------

    def move_to(self, x, y, z, quat, tolerance=0.01):
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05

        constraints = Constraints()
        constraints.name = "physics_pose_goal"

        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = EE_LINK
        position.weight = 1.0
        volume = BoundingVolume()
        volume.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[tolerance] * 3)
        )
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        volume.primitive_poses.append(pose.pose)
        position.constraint_region = volume
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = EE_LINK
        (
            orientation.orientation.x,
            orientation.orientation.y,
            orientation.orientation.z,
            orientation.orientation.w,
        ) = quat
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)

        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        if not self._spin_future(future, self.config.plan_timeout_s * 20):
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveIt rejected the goal")
            return False
        result_future = handle.get_result_async()
        if not self._spin_future(result_future, self.config.plan_timeout_s * 20):
            return False
        return result_future.result().result.error_code.val == 1

    def send_gripper(self, position, max_effort):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        future = self.gripper_client.send_goal_async(goal)
        if not self._spin_future(future, self.config.gripper_action_timeout_s * 20):
            return False
        handle = future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        # A stalled gripper is the expected outcome when squeezing an object,
        # so a timeout here is not by itself a command failure.
        self._spin_future(result_future, self.config.gripper_action_timeout_s * 20)
        return True

    def _spin_future(self, future, wall_timeout):
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return False

    def remove_protection_sphere(self):
        obj = CollisionObject()
        obj.header = Header(frame_id="world", stamp=self.get_clock().now().to_msg())
        obj.id = PROTECTION_ID
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.scene_pub.publish(scene)

    # ---------------- trial ----------------

    def run_trial(self):
        planning_ok = True
        gripper_ok = True
        timed_out = False

        if not self.wait_for_telemetry():
            self.get_logger().error("no /grasp_raw_state; is physics_mode running?")
            return self._finish(False, False, True)

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.set_stage("settle")
        if not self.wait_sim(self.config.settle_duration_s):
            timed_out = True
        with self._lock:
            self._baseline_z = self._latest.object_pose.position.z
        self.get_logger().info("settled baseline z = {:.4f}".format(self._baseline_z))

        deadline = time.time() + 60.0
        while self._target is None and time.time() < deadline:
            time.sleep(0.1)
        if self._target is None:
            self.get_logger().error("no target observation on /apple_location")
            return self._finish(False, gripper_ok, True)
        x, y, z_surface = self._target

        quat = (0.0, 1.0, 0.0, 0.0)  # tool z pointing down
        hover_z = z_surface + self.config.hover_offset_m
        grasp_z = z_surface + self.config.grasp_z_offset_m

        self.set_stage("hover")
        if not self.move_to(x, y, hover_z, quat, tolerance=0.005):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("open")
        if not self.send_gripper(self.config.open_position, -1.0):
            gripper_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)
        self.wait_sim(0.5)

        self.remove_protection_sphere()
        self.wait_sim(0.5)

        self.set_stage("descend")
        if not self.move_to(x, y, grasp_z, quat, tolerance=0.01):
            planning_ok = False
            self.move_to(x, y, hover_z, quat, tolerance=0.02)
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("close")
        if not self.send_gripper(self.config.close_position, 20.0):
            gripper_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        if not self._confirm_bilateral_contact():
            self.set_stage("retreat")
            self.move_to(x, y, hover_z, quat, tolerance=0.02)
            return self._finish(planning_ok, gripper_ok, timed_out)

        with self._lock:
            self._grasp_relative = self._relative
            self._bilateral_confirmed = True

        self.set_stage("lift")
        lift_z = grasp_z + self.config.lift_height_m + 0.01
        if not self.move_to(x, y, lift_z, quat, tolerance=0.005):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("hold")
        hold_completed = self.wait_sim(self.config.hold_duration_s)

        self.set_stage("retreat")
        self.move_to(x, y, hover_z, quat, tolerance=0.02)

        return self._finish(planning_ok, gripper_ok, timed_out, hold_completed)

    def _confirm_bilateral_contact(self):
        """Accept only bilateral contact sustained for the confirmation window."""
        start = self.sim_time()
        window_start = None
        while True:
            now = self.sim_time()
            if now is None:
                return False
            if now - start > self.config.contact_confirm_timeout_s:
                return False
            with self._lock:
                bilateral = getattr(self, "_summary", None) is not None and self._summary.bilateral
            if bilateral:
                if window_start is None:
                    window_start = now
                elif now - window_start >= self.config.confirm_window_s:
                    return True
            else:
                window_start = None
            time.sleep(0.01)

    def _finish(self, planning_ok, gripper_ok, timed_out, hold_completed=False):
        with self._lock:
            ratio = (
                self._hold_bilateral / self._hold_total if self._hold_total else 0.0
            )
            observations = TrialObservations(
                planning_ok=planning_ok,
                gripper_ok=gripper_ok,
                timed_out=timed_out,
                any_contact=self._any_contact,
                bilateral_confirmed=self._bilateral_confirmed,
                contact_lost_during_lift=self._contact_lost_during_lift,
                max_elevation_m=self._max_elevation,
                hold_completed=hold_completed,
                hold_bilateral_ratio=ratio,
                max_drift_m=self._max_drift,
            )

        category = classify_trial(observations, self.config)
        summary = self.recorder.finish(observations, category)
        self.get_logger().info(
            "trial {} -> {} (elevation {:.4f} m, drift {:.4f} m, hold ratio {:.2f})".format(
                self.trial_id,
                category.value,
                observations.max_elevation_m,
                observations.max_drift_m,
                observations.hold_bilateral_ratio,
            )
        )
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsPicker()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        summary = node.run_trial()
        exit_code = 0 if summary and summary["success"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write the MoveIt launch file for physics mode**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/launch/move_group_physics.launch.py" <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # physics_mode semantics: the SRDF declares no object or marker joints, so
    # the robot state monitor no longer reports missing cube_x/y/z joints.
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_moveit_config")
        .robot_description_semantic(file_path="config/openarm_physics.srdf")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"monitor_dynamics": False},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([run_move_group_node])
EOF
```

- [ ] **Step 3: Write the physics-mode bringup launch file**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/launch/run_physics_grasp.launch.py" <<'EOF'
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ["MUJOCO_GL"] = "osmesa"


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)
    model_dir = os.path.join(launch_dir, "..", "model")

    mujoco_xml_file = os.path.join(model_dir, "table_scene_physics.xml")
    urdf_file = os.path.join(model_dir, "table_scene_physics.urdf")
    controller_config = os.path.join(launch_dir, "physics_controllers.yaml")
    detector_script = os.path.join(launch_dir, "apple_detector.py")
    table_script = os.path.join(launch_dir, "spawn_table_scene.py")

    with open(urdf_file, "r") as handle:
        robot_desc = handle.read()

    viewer_render_period = LaunchConfiguration("viewer_render_period", default="0.0166666667")

    node_mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
            {"mujoco_model_path": mujoco_xml_file},
            controller_config,
            {"viewer_render_period": viewer_render_period},
            {"camera_period": 0.1666666667},
            {"grasp_telemetry.object_geom": "physics_object_geom"},
            {"grasp_telemetry.gripper_body": "openarm_left_hand"},
            {"grasp_telemetry.publish_every_n_steps": 10},
        ],
    )

    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    tf_world_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0", "1.2", "0", "0", "0", "1", "world", "camera_base_link"],
        output="screen",
    )

    tf_cam_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "1", "0", "0", "0", "camera_base_link", "camera_optical_frame"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_left_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_arm_controller"],
        output="screen",
    )
    load_right_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_arm_controller"],
        output="screen",
    )
    load_left_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_hand_controller"],
        output="screen",
    )
    load_right_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_hand_controller"],
        output="screen",
    )

    node_detector = ExecuteProcess(
        cmd=["python3", detector_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    spawn_table = ExecuteProcess(
        cmd=["python3", table_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group_physics.launch.py"))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viewer_render_period",
                default_value="0.0166666667",
                description="Simulated seconds between viewer redraws.",
            ),
            node_mujoco,
            node_rsp,
            tf_world_to_cam,
            tf_cam_to_optical,
            node_detector,
            launch_moveit,
            spawn_table,
            load_jsb,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_jsb,
                    on_exit=[load_left_arm, load_right_arm, load_left_hand, load_right_hand],
                )
            ),
        ]
    )
EOF
```

- [ ] **Step 4: Build and confirm the executable is installed**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws && source /opt/ros/humble/setup.bash && source install_repro/setup.bash && colcon build --build-base build_repro --install-base install_repro --packages-select openarm_grasp_physics openarm_moveit_config && ls install_repro/openarm_grasp_physics/lib/openarm_grasp_physics"
```
Expected: `Summary: 2 packages finished` and `physics_picker` listed.

- [ ] **Step 5: Mirror and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add vm_sync/ros2_course_physics && git commit -m "feat: add physics-mode grasp state machine and launch files"
```

---

### Task 13: Integration validation

Every check here is one the software-attached demo would silently pass, so each is run explicitly.

**Files:**
- Create (VM): `~/ros2_ws_course_repro/results/physics_grasp/integration/notes.md`

**Interfaces:**
- Consumes: everything from Tasks 1 through 12.
- Produces: recorded evidence that `physics_mode` is free of attachment mechanisms and that object motion requires contact. No source changes unless a check fails.

- [ ] **Step 1: Launch physics mode and confirm the graph is ready**

Terminal A on the VM:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0 > /tmp/physics_launch.log 2>&1"
```

- [ ] **Step 2: Verify telemetry is live and no attachment path exists**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && ros2 topic list | grep -E 'grasp_raw_state|cube_controller'; echo '--- controllers ---'; ros2 control list_controllers"
```
Expected: `/grasp_raw_state` present, no `cube_controller` topic, and `list_controllers` showing exactly `joint_state_broadcaster`, `left_arm_controller`, `right_arm_controller`, `left_hand_controller`, `right_hand_controller`, all `active`.

- [ ] **Step 3: Verify the object rests on the table with no command**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && ros2 topic echo /grasp_raw_state --once | head -30"
```
Expected: `object_pose.position.z` near `0.445` (sphere radius above the `0.42` table surface) and stable across repeated calls; `contact_geoms` containing `table_top`.

- [ ] **Step 4: Verify the MoveIt cube-joint errors are gone**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "grep -c 'cube_x\|cube_y\|cube_z' /tmp/physics_launch.log"
```
Expected: `0`.

- [ ] **Step 5: Verify a deliberately misaligned close does not move the object**

Run a trial with a lateral offset that guarantees the fingers miss:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && ros2 run openarm_grasp_physics physics_picker --ros-args -p use_sim_time:=true -p trial_id:=misaligned_check -p output_dir:=/home/nostalgia/ros2_ws_course_repro/results/physics_grasp/integration; echo exit=\$?"
```
Before running, temporarily set `grasp_z_offset_m` in `config.py` to `0.05` so the gripper closes above the object.

Expected: `exit=1`, and `results/physics_grasp/integration/misaligned_check/trial.json` showing `"category": "no_contact"` with `max_elevation_m` below `0.002`. This is the check that proves nothing carries the object but contact.

Restore `grasp_z_offset_m` to `0.0` afterwards.

- [ ] **Step 6: Run one aligned trial and inspect the outcome**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && ros2 run openarm_grasp_physics physics_picker --ros-args -p use_sim_time:=true -p trial_id:=aligned_check -p output_dir:=/home/nostalgia/ros2_ws_course_repro/results/physics_grasp/integration; echo exit=\$?"
```
Then:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cat /home/nostalgia/ros2_ws_course_repro/results/physics_grasp/integration/aligned_check/report.md"
```

Expected at this point: a trial that reaches at least `close` and records contact forces. A failure category here is a legitimate research result, not necessarily a defect. Only these count as defects to fix now:
- `gripper_command_failure` caused by the controller never accepting a goal;
- zero samples recorded;
- `no_contact` combined with telemetry showing finger geoms in `contact_geoms` (a classification bug);
- object motion with no contact in the samples (an attachment leak).

Any other category is a physical finding for Step 7.

- [ ] **Step 7: If the grasp does not hold, diagnose before tuning**

Work through the recorded evidence in this order and record the finding in `notes.md` before changing any parameter:

1. `samples.csv` shows no finger contact at all → geometric misalignment; check the descend pose against the finger geometry.
2. Contact on one finger only → coupling or approach-angle problem, not force.
3. Bilateral contact with small normal forces → raise `position_kp` or `position_max_effort`, one at a time, and record the old and new values.
4. Bilateral contact with high force but the object escapes on lift → contact friction; the table geom uses `friction="0.05 0.01 0.01"` and the object geom inherits defaults, so the object/finger friction pair is the parameter to examine.

Every parameter change is recorded with its before/after value and the trial id that motivated it. Physics timestep stays `0.002` regardless.

- [ ] **Step 8: Write the integration notes**

Create `~/ros2_ws_course_repro/results/physics_grasp/integration/notes.md` recording: the controller list, the resting object height, the misaligned-trial category, the aligned-trial category, and every parameter change with its justification.

- [ ] **Step 9: Mirror results and commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && mkdir -p docs/results/physics_grasp && scp -P 2224 -r nostalgia@127.0.0.1:/home/nostalgia/ros2_ws_course_repro/results/physics_grasp/integration docs/results/physics_grasp/ && bash tools/sync_from_vm.sh && git add docs/results/physics_grasp vm_sync/ros2_course_physics && git commit -m "test: record physics-mode integration validation evidence"
```

---

### Task 14: Ten-trial acceptance run and Stage 1 result

**Files:**
- Create (VM): `src/openarm_grasp_physics/scripts/run_acceptance.sh`
- Create (Windows): `docs/results/2026-09-01-openarm-physics-grasp-stage1.md`
- Modify (Windows): `README.md`

**Interfaces:**
- Consumes: everything from Tasks 1 through 13.
- Produces: 10 trial records under `results/physics_grasp/acceptance/`, an aggregate summary, and the Stage 1 result document. Stage 1 passes when at least 8 of 10 trials are `success`.

- [ ] **Step 1: Write the acceptance driver**

```bash
ssh -p 2224 nostalgia@127.0.0.1 "mkdir -p ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/scripts && cat > ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/scripts/run_acceptance.sh" <<'EOF'
#!/usr/bin/env bash
# Ten independent physics_mode grasp trials. Each trial gets a fresh simulator
# so trials cannot inherit state from one another.
set -uo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
export MUJOCO_GL=osmesa

OUT="$HOME/ros2_ws_course_repro/results/physics_grasp/acceptance"
TRIALS="${1:-10}"
mkdir -p "$OUT"

for i in $(seq 1 "$TRIALS"); do
  TRIAL_ID=$(printf "trial_%03d" "$i")
  echo "=== $TRIAL_ID ==="

  ros2 launch openarm_moveit_config run_physics_grasp.launch.py \
    viewer_render_period:=1.0 > "$OUT/$TRIAL_ID.launch.log" 2>&1 &
  LAUNCH_PID=$!

  sleep 25

  ros2 run openarm_grasp_physics physics_picker --ros-args \
    -p use_sim_time:=true \
    -p trial_id:="$TRIAL_ID" \
    -p output_dir:="$OUT" > "$OUT/$TRIAL_ID.picker.log" 2>&1
  echo "$TRIAL_ID exit=$?"

  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  sleep 5
done

python3 - "$OUT" <<'PY'
import json, os, sys, collections
out = sys.argv[1]
counts = collections.Counter()
rows = []
for name in sorted(os.listdir(out)):
    path = os.path.join(out, name, "trial.json")
    if not os.path.isfile(path):
        continue
    with open(path) as handle:
        payload = json.load(handle)
    counts[payload["category"]] += 1
    rows.append((payload["trial_id"], payload["category"],
                 payload["observations"]["max_elevation_m"],
                 payload["observations"]["max_drift_m"],
                 payload["observations"]["hold_bilateral_ratio"]))

summary = {
    "trials": len(rows),
    "successes": counts.get("success", 0),
    "categories": dict(counts),
    "rows": rows,
}
with open(os.path.join(out, "acceptance_summary.json"), "w") as handle:
    json.dump(summary, handle, indent=2, sort_keys=True)
print(json.dumps(summary, indent=2))
PY
EOF
```

Then make it executable:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "chmod +x ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/scripts/run_acceptance.sh"
```

- [ ] **Step 2: Smoke-test the driver with two trials**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/scripts/run_acceptance.sh 2"
```
Expected: a JSON summary with `"trials": 2` and per-trial rows. Fix driver problems (missing logs, zero trials found) before the full run.

- [ ] **Step 3: Run the full acceptance set**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "rm -rf ~/ros2_ws_course_repro/results/physics_grasp/acceptance && ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics/scripts/run_acceptance.sh 10"
```
Expected: `"trials": 10` with a category breakdown.

Stage 1 passes when `"successes"` is at least `8`. If it is lower, return to Task 13 Step 7, record the diagnosis and the parameter change, and re-run. Do not report Stage 1 as passing on a partial result.

- [ ] **Step 4: Re-run the full Python suite as a regression check**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_grasp_physics && python3 -m pytest test -q"
```
Expected: `44 passed`.

Then re-run the existing (read-only) experiment package suite under `openarm_ws` to confirm nothing regressed. This workspace is never edited by this plan; only its tests are executed:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "cd ~/openarm_ws/experiments/openarm_robust_grasp && python3 -m pytest tests -q"
```
Expected: `221 passed`.

- [ ] **Step 5: Confirm the demo baseline still launches**

Run:
```bash
ssh -p 2224 nostalgia@127.0.0.1 "source /opt/ros/humble/setup.bash && source ~/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash && export ROS_DOMAIN_ID=71 && timeout 45 ros2 launch openarm_moveit_config bringup_all.launch.py > /tmp/demo_regression.log 2>&1; grep -c 'cube_controller' /tmp/demo_regression.log"
```
Expected: a non-zero count, showing the original object controller still loads in `demo_mode`.

- [ ] **Step 6: Copy results to the research repo**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && scp -P 2224 -r nostalgia@127.0.0.1:/home/nostalgia/ros2_ws_course_repro/results/physics_grasp/acceptance docs/results/physics_grasp/
```

- [ ] **Step 7: Write the Stage 1 result document**

Create `docs/results/2026-09-01-openarm-physics-grasp-stage1.md` containing:
- what changed relative to the course demo, in one paragraph;
- the acceptance table: trial id, category, max elevation, max drift, hold bilateral ratio;
- successes out of 10 and whether Stage 1 passed;
- every parameter that was tuned, with before and after values and the trial that motivated it;
- the two model corrections (finger coupling `polycoef`, removed finger position actuators) and why each was wrong;
- explicit caveats: arm joints are still position-teleported rather than actuator-driven, so joint velocities during arm motion are not physical; friction and solver parameters are untuned defaults; 10 trials establish repeatability under one fixed condition, not a success probability.

- [ ] **Step 8: Update the repository README**

Replace the `## 下一阶段` section with a `## 当前阶段结果` section stating the Stage 1 outcome and pointing at the new result document and this plan, then describe Stage 2 (calibration error and pre-grasp visual correction) as the next step.

- [ ] **Step 9: Commit**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && bash tools/sync_from_vm.sh && git add -A docs vm_sync README.md && git commit -m "feat: complete Stage 1 physical grasp with ten-trial acceptance evidence"
```

- [ ] **Step 10: Push**

```bash
cd "/c/Users/ASUS/Documents/ChatGPT/open-arm相关" && git push origin main
```

---

## Stage 1 Exit Criteria

Stage 1 is complete when all of these hold:

- no attachment loop, object pose publisher, object controller, or object slide joint exists in any `physics_mode` file;
- `demo_mode` launches and behaves exactly as before;
- the static asset tests pass, proving the above by inspection rather than by claim;
- a misaligned close leaves the object on the table;
- at least 8 of 10 acceptance trials record `success`, each with elevation at least `0.020 m`, a completed `2.0 s` hold, hold bilateral-contact ratio at least `0.80`, and relative drift at most `0.010 m`;
- the full Python suite and the existing experiment suite both pass;
- results, parameter changes, and caveats are written down and committed.

Stage 2 (calibration error and pre-grasp visual correction) begins only after this.
