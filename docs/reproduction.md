# 复现指南

日期：2026-09-06
适用范围：ROS2 物理抓取阶段（`vm_sync/ros2_course_physics/`）与 `tools/experiments/` 下的实验脚本。

---

## 1. 环境

以下是全部已记录结果所在的环境。目前**只有散落在各文档中的文字记载**，没有锁文件；此处集中一遍。

| 项 | 版本 |
|---|---|
| OS | Ubuntu 22.04.5 (VirtualBox guest) |
| Python | 3.10.12 |
| MuJoCo | 3.10.0 |
| ROS 2 | Humble |
| NumPy | 1.21.5 |
| pytest | 6.2.5 |
| PID 实现 | `control_toolbox::Pid` |
| 构建 | colcon（symlink-install） |

依赖：`tools/requirements-sim.txt`（离线 MuJoCo 框架）。ROS2 侧依赖由 `rosdep` 与各包 `package.xml` 决定。

**注意**：上游 OpenArm MuJoCo 资产以绝对路径引用（`~/openarm_ws/src/openarm_mujoco`），未随仓库分发，许可证未逐项确认——见 README「来源与许可」。

## 2. 构建

```bash
cd ~/ros2_ws_course_repro/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --build-base build_repro --install-base install_repro
```

`--symlink-install` 是有意选择：Python 节点、launch 文件与模型 XML 的改动**无需重新构建**即生效；只有 C++（`mujoco_ros2_control`）改动需要重建。

测试：

```bash
colcon test --build-base build_repro --install-base install_repro \
  --packages-select openarm_grasp_physics && colcon test-result --verbose
```

现有 52 项。

## 3. 运行

所有实验脚本自解析所在目录，配置集中在 `tools/experiments/env.sh`，默认值即原 VM 设置。换机器只需覆盖环境变量：

```bash
export OPENARM_ROOT=/path/to/ros2_ws_course_repro
export ROS_DOMAIN_ID=71
bash tools/experiments/run_rel.sh trial01 0.0 0.240 mybatch -0.0209 -0.0007
```

参数：`trial_id`、`CAM_BIAS_X`、`CAM_BIAS_Y`（米）、输出批次名、`rel_offset_x`、`rel_offset_y`。

一次扫描：

```bash
bash tools/experiments/sweep_ydense.sh
```

**每次试验一次全新 launch。** 复用同一实例会让上一次试验停在退避位姿的手臂遮挡相机——这是 vision 步骤 2/3 曾产出冻结读数的原因。`run_fast_batch.sh` 复用实例，脚本头部有明确警告。

## 4. 复现性的已知界限

**统计可复现，非逐位可复现。**

- MoveIt 使用 RRTConnect，是随机采样规划器，**当前未设种子**。多份文档把结果翻转归因于它（工作区覆盖文档记录 5 个复测点中有 3 个翻转）。
- MuJoCo 侧未设种子；`trial.json` 无 seed 字段。
- 唯一的 RNG 是 `physics_picker.py` 中由 `trial_id` 字符串播种的 ±1mm 合成视觉噪声。

因此：**重跑 N 次应得到相近成功率，但不会得到相同的单次结果。**离线框架（`vm_sync/experiments/openarm_robust_grasp/`）反而是有种子且配对的（`run_experiments.py` 的 `paired_seed`）。为 ROS2 侧补种子是计划中的 B1 项。

每次试验记录三份产物：`samples.csv`、`trial.json`（含**完整配置快照**，约 35 个参数）、`report.md`。批次配置可从数据本身恢复——下表即由 `trial.json` 生成，非人工记录。

## 5. 实验清单

由 `docs/results/physics_grasp/*/trial.json` 的配置签名自动生成。"方案"列直接读自记录，不依赖文档描述。

| batch | n | 方案 | 首个 trial_id |
|---|---|---|---|
| `fastbatch` | 6 | open-loop | `fb01` |
| `integration` | 2 | open-loop | `aligned_check` |
| `stage1_formation` | 5 | open-loop | `f01` |
| `stage1_acceptance` | 10 | open-loop | `a01` |
| `stage2` | 27 | ideal-correction + open-loop | `ol_b12_r1` |
| `stage2b` | 18 | open-loop | `nx008_r1` |
| `stage2c` | 12 | open-loop | `r01` |
| `stage2_vision` | 19 | detector-target | `v2_b000_1` |
| `stage3_acceptance` | 10 | open-loop | `s01` |
| `stage3_closedloop` | 8 | naive-redetect | `cl_b000_1` |
| `stage4_relative` | 29 | relative(off=-2.1,-5.0mm) + relative(off=0,0) | `rel_b028_1` |
| `stage5_perspective` | 4 | relative(off=0,0) | `pc_b192_z1` |
| `stage6_xy` | 10 | relative(off=-2.1,-5.0mm) | `xy_d032_1` |
| `stage7_xgain` | 6 | relative(off=-2.1,-5.0mm) | `xg_px064_1` |
| `stage8_randrel` | 12 | relative(off=-2.1,-5.0mm) | `rr01` |
| `stage9_randxy` | 12 | relative(off=-2.1,-5.0mm) | `dr01` |
| `stage10_blobdiag` | 9 | relative(off=-2.1,-5.0mm) | `bd01` |
| `stage11_deep` | 15 | relative(off=-2.1,-5.0mm) + relative(off=-20.9,-0.7mm) | `dp01` |
| `stage12_limits` | 6 | relative(off=-20.9,-0.7mm) | `lim_x096_1` |
| `stage13_outlier` | 12 | relative(off=-20.9,-0.7mm) | `om01` |
| `stage14_p88` | 16 | relative(off=-23.2,20.5mm) + relative(off=0,0) | `p88cal1` |
| `stage15_roi` | 19 | relative(off=-23.3,-2.2mm) + relative(off=0,0) | `roib1` |
| `stage16_limits2` | 23 | relative(off=-20.9,-0.7mm) + 另两组标定 | `fb_x096_1` |
| `stage17_yaw` | 6 | relative(off=-20.9,-0.7mm) | `reach112_1` |
| `stage18_errcode` | 9 | relative(off=-20.9,-0.7mm) | `ec01` |
| `stage19_ycliff` | 27 | relative(off=-20.9,-0.7mm) | `yd208_1` |
| `stage20_workspace` | 23 | relative(off=-20.9,-0.7mm) | `ws2_x25_yn20` |
| `stage21_reachcal` | 6 | relative(off=-20.9,-0.7mm) | `cal_x25y00_1` |
| `stage22_offsetmap` | 9 | relative(off=0,0) | `om_x25y00_1` |
| `stage23_feature` | 8 | relative(off=0,0) | `fc01` |
| `stage24_roi30` | 12 | relative(off=0,0) | `r30_01` |
| `stage25_marker` | 5 | relative(off=0,0) | `hist_trial` |
| `stage26_cascade` | 21 | relative(off=0,0) | `cas0_1` |
| `stage27_joints` | 5 | relative(off=0,0) | `jr01` |
| `stage28_branch` | 8 | relative(off=0,0) | `br01` |
| `stage29_settle` | 6 | relative(off=0,0) | `st01` |
| `stage30_band` | 6 | relative(off=0,0) | `bd01` |
| `stage31_depthcheck` | 8 | relative(off=0,0) | `dc01` |
| `stage32_best` | 10 | relative(off=0,0) | `bs0_1` |
| `stage33_final` | 6 | relative(off=0,0) | `fin0_1` |
| `vision_test` | 5 | detector-target | `vision01` |

**合计 470 次。**

`relative(off=0,0)` 的批次是**诊断批**（测量残差本身），不是抓取评测批——未减标定常数，成功率不可与评测批并列比较。这一区分此前只存在于各文档正文里，未在数据层面标注。

## 6. 尚未解决

- 无 `requirements.txt` 之外的锁文件、无 Dockerfile、无 vcs repos 文件。
- 上游资产未 vendored，许可证未确认（**公开前必须解决**）。
- 无种子（见 §4）。
- `trial.json` 未记录 commit hash 与资产校验和。全仓库唯一做过此事的是 `docs/results/2026-08-22-simulation-grasp-stability.md`（记录了 scene SHA-256）。
