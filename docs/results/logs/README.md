# 实验日志归档

## 保留策略

每次试验产出四类记录：

| 产物 | 位置 | 内容 |
|---|---|---|
| `trial.json` | `docs/results/physics_grasp/<批次>/<试验>/` | 配置快照（约 35 项）、结果分类、阶段耗时、18 项光学测量、环境块 |
| `samples.csv` | 同上 | 逐帧物理量（约 1500–1900 行/次） |
| `report.md` | 同上 | 人读摘要 |
| **`launch.log.gz` / `picker.log.gz`** | 同上（**2026-09-08 起**） | 探测器逐帧输出与级联轨迹、MoveIt 错误码、picker 的修正推理 |
| 批次级 sweep 日志 | `docs/results/logs/` | 每次试验的判定行与 `INSTABILITY` 计数 |

## 已发生的损失（2026-09-08 记录）

`run_rel.sh` 原先把 launch 与 picker 日志写在虚拟机 `/tmp`。**`/tmp` 在每次重启时清空**，因此以下批次的**逐次**日志已不可恢复：

| 批次 | 内容 | 状态 |
|---|---|---|
| `stage35_offsetgeom` | x 轴，35 次 | 逐次日志已丢 |
| `stage36_offsetgeom_y` | y 轴未修正，47 次 | 逐次日志已丢 |
| `stage37_widecascade` | 宽首遍对照，24 次 | 逐次日志已丢 |
| `stage38_viewgain` | 单增益，32 次 | 逐次日志已丢 |

**未丢的**：这四批的 `trial.json`、`samples.csv`、`report.md` 全部完整（它们一直写在结果目录），批次级 sweep 日志亦完整（写在 `~`，不受 `/tmp` 清空影响）。

因此各文档中引用的数值——残差、深度、`view_tan`、成功率、稳定性校验——**全部仍可从已提交数据重算**。丢失的是逐次的探测器级联轨迹与 picker 推理过程，属诊断细节，不支撑任何已发表结论。

## 修复

`run_rel.sh` 自 2026-09-08 起把两份日志写入该次试验自己的目录并 gzip（原始约 76KB，压缩后不到 10KB，整批不到 1MB）。同时，**在记录器写出 `trial.json` 之前就失败的试验**——预检中止、启动失败——现在也会留下一个只含日志的目录，此前这类失败不留任何痕迹。

## 批次级日志对照

| 文件 | 批次 |
|---|---|
| `og_cascade.log` | `stage35_offsetgeom`（x 轴，级联） |
| `og_y.log` | `stage36_offsetgeom_y`（y 轴，未修正） |
| `og_wide.log` | `stage37_widecascade`（宽首遍对照） |
| `og_vg.log` | `stage38_viewgain`（仅夹爪增益） |
| `og_bg.log` / `og_bg2.log` | `stage39_bothgains`（双增益；关机截断后补跑） |
