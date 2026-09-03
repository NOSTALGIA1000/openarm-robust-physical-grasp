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

## 当前阶段结果

**Stage 1 与 Stage 3 均已通过。系统可完成真实物理抓取并抬起物体。**

- **假绑定已彻底证伪**：错位对照试验 824 采样零接触、物体全程未动；且实测原课程夹爪开口 72 mm 大于 50 mm 物体，本就碰不到，其「抓取」100% 来自软件绑定。
- **Stage 1（抓取形成）10/10 通过**：双侧夹持形成 10/10，hold 双接触率 1.00 共 10/10，零发散。
- **Stage 3（执行器级力控 + 搬运）10/10 通过**：抬升均值 26.4 mm（要求 ≥20）、漂移均值 7.2 mm（要求 ≤10）、hold 双接触率全部 1.00、零发散。

抬升成立需要三个条件同时满足：位置指令速度一致（`qvel`）、手臂关节具备转子惯量（`armature`）、执行器级力控（`position_pid` + 力矩钳位）。缺任一条物体都不会离开桌面。

结论文档：[Stage 3](docs/results/2026-09-03-openarm-stage3-result.md) · [Stage 1](docs/results/2026-09-03-openarm-stage1-result.md) · [机制调查](docs/results/2026-09-02-openarm-grasp-mechanics-findings.md)

## 下一阶段

3. Stage 2：注入手眼标定误差与视觉噪声，比较开环抓取与抓取前视觉修正。

## 来源与许可

项目包含实验代码、实验结果，以及基于 OpenArm/MuJoCo 资产形成的研究材料。上游组件保留各自许可。本仓库暂不声明覆盖全部内容的统一许可证；公开复用前需逐项确认上游和课程参考代码许可。
