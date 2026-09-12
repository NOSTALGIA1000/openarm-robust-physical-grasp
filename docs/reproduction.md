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

**注意**：上游 OpenArm MuJoCo 网格资产未随仓库分发。模型用 `meshdir="meshes"` 相对路径解析到本机 OpenArm checkout（`~/openarm_ws/src/openarm_mujoco`）。许可已逐项核查完毕，见 [THIRD_PARTY.md](../THIRD_PARTY.md)。

### 1.2 模型校验和在 2026-09-12 变过一次（仅注释）

`trial.json` 的 `environment.model_sha256` 记录模型文件的 sha256。2026-09-12 为满足 Apache-2.0 §4 给 `openarm_bimanual_physics.xml` 恢复上游版权头并附修改声明，**该文件的哈希因此改变**：

| | sha256（前 16 位） |
|---|---|
| 2026-09-12 之前 | `257722e8a00e698c` |
| 之后 | `9846f6858142d5f0` |

**模型物理上未变**，改动只有文件开头的 XML 注释。已用 MuJoCo 真编译核对前后一致：

```
table_scene_physics.xml   nbody=36 njnt=22 ngeom=110 nsensor=5
table_scene_c020.xml      nbody=36 njnt=22 ngeom=110 nsensor=5
```

复核：

```bash
ssh -p 2224 nostalgia@127.0.0.1 'cd ~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model &&   python3 -c "import mujoco; m=mujoco.MjModel.from_xml_path(\"table_scene_physics.xml\");   print(m.nbody, m.njnt, m.ngeom, m.nsensor)"'
```

跨 2026-09-12 比较批次时，**哈希不同不代表模型不同**；反之哈希相同仍是模型相同的充分证据。

### 1.1 虚拟机配额与内核软锁死

| 项 | 值 |
|---|---|
| 宿主逻辑核 | 16 |
| VM vCPU | **4**（`cpuexecutioncap=100`）|
| VM 内存 | 7675 MB |
| 跑批时 load average | **约 4.3**（即 4 核满载）|

**4 个 vCPU 对本流水线不够用。**MuJoCo、`ros2_control`、`move_group`、探测器与 picker 同时在跑，guest 内核的 RCU 宽限期线程被饿死，产生软锁死：

```
watchdog: BUG: soft lockup - CPU#2 stuck for 179s! [swapper/2:0]
rcu: rcu_preempt kthread starved for 200356 jiffies!
```

两天内 **6 次**同类内核事件（`journalctl -k --since "2026-09-10" | grep -c "rcu_preempt\|soft lockup"`）。**这就是此前记为「ROS CLI 挂死」的那四次事件的机制**（105 分钟、11.6 小时、3 小时，以及 `stage53_noise20` 的 `dn_n000_y128_11` 挂 28 分钟后被下一次试验的 preflight 用 SIGKILL 清掉——所有进程 `exit code -9`，正是设计中的清理行为）。

**后果与处置**：

- **物理结论不受影响**：仿真按 sim_time 推进，每个批次都断言 `sim_time` 严格单调并 grep `unstable`/`QACC`；受影响的是墙钟编排，不是动力学。
- **试验丢失率约 4%**（`stage53_noise20` 48 次丢 2 次）。样本量规划要按此打折：要 n=27 就得排 29 次。
- 丢失的试验**不留 `trial.json`**，只留 `launch.log`。因此「目录数」不等于「试验数」，分析脚本必须按 `trial.json` 存在与否计数——不是按目录。
- **建议把 vCPU 提到 8**（宿主有 16 核，未超配）。需要关机改配置，**不要在批次运行时动**。


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
