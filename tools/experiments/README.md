# 实验装置

这些脚本原本只存在于 VM 的 `/tmp`，一次重启就会让所有已发表结果无法复现。现纳入版本控制。

## 配置

所有脚本自解析所在目录（`EXPDIR`）并 source `env.sh`。此前脚本互相以 `/tmp/xxx.sh` 调用、路径写死在 19 个文件里，意味着**从 checkout 里根本跑不起来**——`sweep_ydense.sh` 曾是这个问题最直接的证据。

`env.sh` 的每个变量都以原 VM 设置为默认值，换机器覆盖环境变量即可：

| 变量 | 默认 |
|---|---|
| `OPENARM_ROOT` | `$HOME/ros2_ws_course_repro` |
| `OPENARM_WS` / `OPENARM_INSTALL` | `$OPENARM_ROOT/ros2_ws` / `.../install_repro` |
| `OPENARM_RESULTS` | `$OPENARM_ROOT/results/physics_grasp` |
| `OPENARM_MODEL` / `OPENARM_SHARE` | 源模型目录 / 安装后的 share 目录 |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` |
| `MUJOCO_DIR` / `DISPLAY` / `ROS_DOMAIN_ID` | `$HOME/.local/...`/`:0`/`71` |

完整复现步骤见 [`docs/reproduction.md`](../../docs/reproduction.md)。

## `experiments/` — 真实流程运行器

| 脚本 | 用途 |
|---|---|
| `env.sh` | 共享配置（上表） |
| `preflight.sh` | 每次试验前的资源守卫：清理陈旧仿真进程、重启 ros2 CLI daemon、清 FastRTPS 共享内存、内存低于下限则中止 |
| `run_grip_trial.sh` | 单次试验：全新 launch → 校验并修复控制器状态 → 跑 picker → 报告发散计数 |
| `run_batch.sh` | 批量验收，含 `sim_time` 单调性与 QACC 校验 |
| `run_s2.sh` / `sweep_s2.sh` | Stage 2 标定偏置与视觉修正扫描 |
| `run_record.sh` | 录制真实抓取时刻的关节角（离线测试台需要） |
| `run_rel.sh` / `run_randrel.sh` | 光学相对修正（固定位姿 / 随机位姿） |
| `sweep_ydense.sh` | y 向 240–280mm 加密，带 240/256 锚点 |
| `demo.sh` | 开环 vs 相对修正并排演示 |

**每次试验必须独立 launch。** 场景无 reset 服务，物体位姿只能靠重启复位；共用实例会让前次试验的位移污染后续。

## `harness/` — 离线 MuJoCo 测试台

脱离 ROS/MoveIt，每次几秒而非几分钟。

**关键教训：测试台必须先能复现真实失败，才能用来做参数决策。** 最初版本把物体悬空放在夹爪中心，给出 carry 0.63–0.79，据此两次推荐参数改动、两次都没转移到真实系统。改成用真实抓取关节角、物体**坐在桌面上**（`harness2.py`）之后，它立刻复现真实失败（carry −0.02 对真机 0.02），此后结论全部转移。

「物体被托在自由空间」与「物体静置于支撑面」是两个不同的力学问题：后者物体可以原地不动、手指从其表面滑过，根本不构成对夹持的检验。

| 脚本 | 用途 |
|---|---|
| `harness2.py` | **有代表性**的测试台（真实抓取姿态 + 物体在桌上） |
| `armtest.py` | 转子惯量扫描——定位到 Stage 3 的根因 |
| `pid_rep.py` | 执行器级 PID 增益扫描 |
| `contact_sweep.py` / `sweep3.py` / `margin_test.py` | 接触参数扫描 |
| `normals.py` | 检查接触法向与穿透距离 |
| `contact_probe.py` / `record_joints.py` | 运行时探针（接触 geom 名、关节角） |
| `stage3_probe.py` | 早期非代表性测试台，保留作对照 |
