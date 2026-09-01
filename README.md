# OpenArm Robust Physical Grasp

面向 OpenArm 机械臂的仿真优先研究项目。当前目标：把课程演示中的软件绑定抓取升级为可测量的 MuJoCo 接触动力学抓取，再研究手眼标定误差与视觉闭环修正。

## 当前成果

- OpenArm MuJoCo 抓取实验框架；
- 可复现随机种子与成对策略比较；
- 接触、双指同帧接触、抬升和保持阶段指标；
- 36 条件稳定性筛查；
- 接触形成与夹爪表面几何诊断；
- ROS2 + MuJoCo 课程流程复现记录；
- 下一阶段真实物理夹取设计规范。

现有诊断结论：旧官方 v2 旋转夹爪模型未形成持续双侧接触；课程流程中的滑动夹爪更接近实验室真机，但当前物体由三轴控制器和软件跟随移动，不能作为真实物理抓取证据。

## 目录

- `vm_sync/experiments/openarm_robust_grasp/`：MuJoCo 实验代码、测试、场景和结果。
- `docs/results/`：实验结论与复现截图。
- `docs/superpowers/specs/`：研究设计规范。
- `docs/superpowers/plans/`：分阶段实施计划。

## 测试

VM 环境：Ubuntu 22.04、Python 3.10、MuJoCo 3.10.0。

```bash
cd vm_sync/experiments/openarm_robust_grasp
python3 -m pytest tests -q
```

## 下一阶段

实施 `physics_mode`：

1. 物体改为自由刚体；
2. 删除 `cube_x/y/z` 控制和 `teleport_cube()`；
3. 滑动夹爪改为受限动力学闭合；
4. 发布双指接触、接触力、物体抬升和相对漂移；
5. 完成 `20 mm` 抬升、`2 s` 保持、10 次至少成功 8 次的验收；
6. 通过后加入标定误差和抓取前视觉修正。

设计规范：[`docs/superpowers/specs/2026-09-01-openarm-ros2-physical-grasp-design.md`](docs/superpowers/specs/2026-09-01-openarm-ros2-physical-grasp-design.md)

## 来源与许可

项目包含实验代码、实验结果，以及基于 OpenArm/MuJoCo 资产形成的研究材料。上游组件保留各自许可。本仓库暂不声明覆盖全部内容的统一许可证；公开复用前需逐项确认上游和课程参考代码许可。
