# OpenArm 视觉闭环 第十步：hover 规划失败是硬性可达限制，不可解

日期：2026-09-05
状态：**结论为阴性且确定。x 向 `planning_failure` 源于工作空间边界，放宽姿态约束无效。**

原始数据：`docs/results/physics_grasp/stage17_yaw/`（6 次，全部 `sim_time` 单调）
前置：[第九步（上限重测）](2026-09-05-openarm-vision-in-loop-step9-limits.md)

---

## 1. 诊断：目标位姿无解，不是路径问题

第九步发现 x=128 mm 处三次全部 `planning_failure` 且相对修正从未执行。MoveIt 日志给出确切原因：

```
[ompl]: RRTConnect.cpp:265 - left_arm/left_arm: Unable to sample any valid states for goal tree
[ompl]: ParallelPlan::solve(): Unable to find solution by any of the threads in 31.02 seconds
[move_group]: Catastrophic failure
```

`Unable to sample any valid states for goal tree` 意味着**目标位姿本身没有有效 IK 解**（或全部解碰撞），与路径搜索无关。目标为 hover 在 (0.4286, 0.0013, 0.5165)。

## 2. 假设：姿态约束过严——已被证伪

原姿态约束三轴均为 0.01 rad（0.57°）。其中绕工具 z 轴的分量在物理上是多余的：**平行钳口对称，且 Stage 2 已证实物体在载荷下自对齐**，把 yaw 钉死只会白白牺牲可达状态。

因此加入 `hover_yaw_tolerance` 参数，在同一 x=128 mm 目标上对比：

| yaw 容差 | IK 采样失败次数 | 结果 |
|---|---|---|
| 0.01 rad（0.57°） | 4 | `planning_failure` |
| 0.31 rad（18°） | — | 无效试验（见下） |
| **1.57 rad（90°）** | **4** | 仍失败，400 s 超时 |

**0.57° 与 90° 的 IK 失败次数完全相同。姿态约束不是原因，假设被否定。**

`yaw=0.31` 那次是无效试验：日志显示它连 `stage -> hover` 都没打印，也没有 vision target，卡在目标获取阶段（探测器未报告），属基础设施失败，根本没走到规划。**其「IK 失败 0 次」曾被误读为「放宽 yaw 使 IK 有解」，实为从未尝试规划。** 单看某一项计数而不看流程走到哪一步，会得出相反结论。

## 3. 可达边界

| x 偏置 | 目标 x | 结果 |
|---|---|---|
| 96 mm | 0.396 | **3/3 success**（残差 −8.1 ~ −10.9 mm） |
| **112 mm** | **0.412** | **0/3，全 `planning_failure`** |
| 128 mm | 0.428 | 0/4，全 `planning_failure` |

**左臂在该 hover 姿态下的 x 可达极限约为 0.40 m。** 边界落在 96–112 mm 偏置之间。

## 4. 结论：这一项无法「解决」

原三步计划的第 2 项是「解决大偏置下的 hover 规划失败」。**在 x 向它不可解**——不是参数没调对，是机械臂够不到。

由此确定：

- **x 向感知上限永远测不到**（在当前场景布置下）。感知在 96 mm 处残差仅 −8~−11 mm、余量充足，但 112 mm 已超出可达范围。要测得真实感知上限，必须改变场景——把物体移近基座，或改用右臂。
- **x 向「96 mm」是运动学数字，不是感知数字**。第五步至第九步凡以 x 向上限论证感知能力的表述，都应如此限定。

`hover_yaw_tolerance` 参数保留，默认仍为 0.01 rad。它在本问题上被证伪，不因「已经加了」就改默认值。

## 5. 未决

- **y 向的偶发 `planning_failure`**（y=128、y=256 各一次）是另一回事：偶发而非系统性，未诊断。y=192 mm 是 3/3，说明 y 向在该量级尚未触及可达边界。
- y 向 192–256 mm 之间的感知断崖仍未加密。
- 若要继续推进 x 向，需重新布置场景，那会使先前所有 x 向数据失去可比性。

---

## 6. 追加：y 向偶发失败是两类，且其中一类是分类错误

诊断 y 向偶发 `planning_failure` 时发现它们并非同一回事：

| trial | 走到哪一步 | MoveIt 日志 |
|---|---|---|
| `fb_y256_1`、`dp01` | **descend 阶段**失败 | `Solution was found` ×3（hover 与 rehover 都成功） |
| `om09`、`fb_y128_1` | **连 hover 都没进** | 无任何记录 |

后一类根本没到规划器：探测器未报告 → `use_detector_target` 检查失败 → `_finish(False, ...)` → `planning_ok=False` → 归类 `planning_failure`。**感知未就绪被记成了运动学失败。**

### 污染范围

统计全部标为 `planning_failure` 的 20 次试验（依据 picker 日志是否出现 `stage -> hover`）：

- **真规划失败 15 次**
- **实为探测器未报告 5 次**（`om09`、`p88e01`、`p88e02`、`fb_y128_1`、`yaw031`）

需要修订的结论：

- 第九步「y=128 mm 2/2，另 1 次 `planning_failure`」——那次（`fb_y128_1`）实为**感知未就绪**，应记为 2/2 加 1 次基础设施失败；
- §1–§4 的 x 向结论**不受影响**：`reach112_*`、`fb_x128_*`、`yaw001` 全部是真规划失败；
- 第八步 88 百分位那批的 4 次 `planning_failure` 中 2 次属此类，但该方案被否定依据的是残差双峰，结论不变。

### 已修

`FailureCategory` 新增 `DETECTOR_TIMEOUT`，`TrialObservations` 新增 `detector_ok`（默认 `True`，历史记录与既有调用语义不变），`classify_trial` 优先判定该项——没拿到观测的试验，谈不上规划成败。新增 3 个测试覆盖该分支（52/52 通过）。

### 尚未诊断

`descend` 阶段的偶发失败（`fb_y256_1`、`dp01`）：hover 与 rehover 均成功，仅 descend 无解。descend 的位置容差为 0.003 m、姿态 0.01 rad，比 hover 的 0.005 m 更严，是可疑点，但未验证。

---

## 7. 追加：descend 偶发失败——排除两项假设，未能复现

原始数据：`docs/results/physics_grasp/stage18_errcode/`（9 次，全部 `sim_time` 单调）

### 7.1 两项假设被否定

**假设一：descend 的位置容差 0.003 m 过严（§5 曾列为可疑点）。** 查 `dp01`、`fb_y256_1` 的 MoveIt 日志，两次 descend 都走到了

```
Starting trajectory execution ...
left_arm_controller started execution
Goal reached, success!
Completed trajectory execution with status SUCCEEDED
```

**规划与执行都成功。** 至少「规划不出来」不成立。

**假设二：OOM 杀死仿真器。** 两次日志末尾都有 `mujoco_ros2_control: process has died, exit code -9`（SIGKILL），一度据此判定为内存不足。**该判断是把结果当成了原因**：SIGKILL 出现在 trial 判定**之后**，来自下一次试验 `preflight.sh` 的 `kill -9`，属正常清理；且实测内存余量 4.3 GB、`dmesg` 无 OOM 记录。

### 7.2 真正卡住诊断的地方

`Completed trajectory execution with status SUCCEEDED` 来自 `trajectory_execution_manager`，而 `move_to` 判定依据是 action 的 `error_code.val == 1`。**两者可以不一致**——轨迹执行完毕仍可能返回 `GOAL_TOLERANCE_VIOLATED(-6)` 或 `CONTROL_FAILED(-4)`。而该错误码从未被打印，只能从时间戳反推，反推不出结论。

已在 `move_to` 中记录错误码与超时，下次发生时直接给出判定依据：

```python
code = result_future.result().result.error_code.val
if code != 1:
    self.get_logger().error(
        "move_to failed with MoveIt error_code {} at ({:.4f}, {:.4f}, "
        "{:.4f}) tol={:.4f}".format(code, x, y, z, tolerance))
```

### 7.3 未能复现

同条件跑 10 次：7 次 success、1 次 `unilateral_contact`、1 次 `detector_timeout`、1 次 400 s 超时。**零次 descend 失败，因此没有错误码可读。**

合并历史数据，发生率约 **2/30 ≈ 6.7%**。10 次抓不到该事件的概率约 50%，**未复现属正常，不构成「问题已消失」的证据**。要稳定捕获需 30–50 次（约 1.5 小时）。

附带验证：`ec01` 被正确归类为 `detector_timeout`——上一节的分类修复在真实批次中生效，换作以前会被记成 `planning_failure`。

### 7.4 结论

- descend 偶发失败**不是**规划失败，**不是** OOM；
- 真实原因待其下次发生时由 `error_code` 直接给出；
- 影响面有限：发生率约 6.7%，且已确认与感知无关，不影响容差结论。
