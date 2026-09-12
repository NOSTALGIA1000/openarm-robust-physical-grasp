# 第三方组件与许可

日期：2026-09-12
核查方式：逐项读本机上游 checkout 与课程原始压缩包的许可声明，非凭记忆。复核命令见每节末。

本仓库整体采用 **Apache License 2.0**（见 [LICENSE](LICENSE)），与两个自有 ROS 包 `package.xml` 中已声明的一致。上游组件各自保留其许可，归属见 [NOTICE](NOTICE)。

---

## 1. 逐项结论

| 组件 | 上游 | 许可 | 可再分发 | 本仓库处置 |
|---|---|---|---|---|
| `mujoco_ros2_control`（`mujoco_system.*`、`mujoco_ros2_control_node.cpp`、`CMakeLists.txt`、`package.xml`） | [sangteak601/mujoco_ros2_control](https://github.com/sangteak601/mujoco_ros2_control) | MIT | 是 | **已合规**：每个派生文件头部即 MIT 全文（版权行 + 许可授予 + 免责声明），MIT 的唯一要求已满足 |
| `mujoco_grasp_telemetry.*`、`mujoco_scene_reset.*`（放在上述包内） | 本项目自有 | Apache-2.0 | 是 | 已加本项目头部 |
| `openarm_bimanual_physics.xml` | Enactic [`openarm_mujoco`](https://github.com/enactic/openarm_mujoco) `v2/openarm_bimanual.xml` | Apache-2.0 | 是 | **曾缺失**上游版权头；已恢复并附修改声明 |
| `table_scene_physics.urdf` | Enactic [`openarm_description`](https://github.com/enactic/openarm_description) | Apache-2.0 | 是 | 已加署名与修改声明 |
| `openarm_physics.srdf` | MoveIt Setup Assistant 针对 OpenArm 的产物 | Apache-2.0 | 是 | 已加署名 |
| OpenArm STL 网格 | Enactic | Apache-2.0 | 是，但**本仓库不打包** | 模型用 `meshdir="meshes"` 相对路径解析到本机 OpenArm checkout；见 [复现说明](docs/reproduction.md) |
| `apple_detector.py` | 课程 `openarm_moveit_config`，作者 yhang | **`<license>BSD</license>`，版本未指明** | **未决** | 见 §2 |
| `openarm_grasp_physics`、`openarm_grasp_msgs`、`openarm_robust_grasp`、`tools/`、`docs/` | 本项目自有 | Apache-2.0 | 是 | — |
| MuJoCo、ROS 2 Humble、MoveIt 2、`ros2_control`、OpenCV、NumPy | 各自上游 | Apache-2.0 / BSD | 运行期依赖，不打包 | 无需处置 |

复核命令：

```bash
grep -h "<license>" vm_sync/ros2_course_physics/src/*/package.xml vm_sync/ros2_course_physics/src/*/*/package.xml
head -20 vm_sync/ros2_course_physics/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp
ssh -p 2224 nostalgia@127.0.0.1 'head -4 ~/openarm_ws/src/openarm_mujoco/LICENSE; grep -m1 -i copyright ~/openarm_ws/src/openarm_description/package.xml'
```

---

## 2. 唯一未决项：`apple_detector.py`

### 事实

课程压缩包 `~/ros2_ws_course.zip` 内 `ros2_ws/src/openarm_moveit_config/package.xml` 声明：

```xml
<maintainer email="1936951548@qq.com">yhang</maintainer>
<license>BSD</license>
```

该包**没有** `LICENSE`/`COPYING` 文件，源文件内**没有**版权行。因此：

- BSD 的版本（2-clause 还是 3-clause）无法确定；
- BSD 的两个版本都要求源码再分发时"保留上述版权声明与免责声明"，而上游**不存在**可供保留的声明文本与版权行——无法履行一个指向空集的义务，也不能代作者编造。

### 我们的文件确实是派生作品

不是独立重写。量化：

| 指标 | 值 |
|---|---|
| 课程原版行数 | 140 |
| 当前版本行数 | 736 |
| 原版唯一有效行（去空行与注释） | 87 |
| 其中仍存在于当前版本 | **80** |

可追溯的具体内容：针孔反投影块（`x_cam = (u - cx) * d / fx`）、`fy = (height/2)/tan(fovy/2)` 的内参构造、`lookup_transform('world', 'camera_optical_frame')` + `do_transform_point` 的 TF 路径、"首次锁定后持续广播"的状态机、以及类名 `UniversalDetector` 与节点名 `universal_detector_node`。

复核命令：

```bash
ssh -p 2224 nostalgia@127.0.0.1 'cd /tmp && unzip -o -q ~/ros2_ws_course.zip \
  "ros2_ws/src/openarm_moveit_config/launch/apple_detector.py" -d courseorig
o=/tmp/courseorig/ros2_ws/src/openarm_moveit_config/launch/apple_detector.py
m=~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/launch/apple_detector.py
comm -12 <(grep -v "^\s*$\|^\s*#" $o | sed "s/^\s*//" | sort -u) \
         <(grep -v "^\s*$\|^\s*#" $m | sed "s/^\s*//" | sort -u) | wc -l'
```

### 两条出路

1. **联系作者确认**（推荐）：请 yhang 指明 BSD-2 还是 BSD-3 并给出版权行，写入本文件与文件头。一封邮件的成本，且保留诚实的沿革记录。
2. **重写那 80 行**：全是针孔投影与 tf2 的教科书写法，独立重写在技术上不难。但为规避署名而重写是洗来源，不是解决来源——只有在联系不到作者时才走这条。

**在其一完成之前，本仓库不得公开发布。** 其余所有组件均已合规。

---

## 3. 本项目自身的许可选择

选 Apache-2.0，理由：

- 两个自有 ROS 包的 `package.xml` 早已声明 `Apache-2.0`，改成别的会与已发布的包元数据冲突；
- 入站的 MIT 与 BSD 均可并入 Apache-2.0 出站；
- 最大的上游依赖（OpenArm、MuJoCo）本身即 Apache-2.0，同许可省掉兼容性论证；
- 含显式专利授权条款，与本项目的专利方向（见 [相关工作](docs/related-work.md)）不矛盾：Apache-2.0 授出的是**本作品所含贡献**的专利许可，不妨碍就本仓库之外的机制申请专利，但意味着**写进本仓库并以 Apache-2.0 发布的机制，其专利许可已授予使用者**。若确定要走专利，应先申请后开源，或把待申请部分留在仓库之外。

---

## 4. 数据与文档

`docs/results/**` 下的试验记录（`trial.json`、CSV、PNG）为本项目生成的实验数据，随仓库以 Apache-2.0 发布。其中不含任何上游资产的拷贝。
