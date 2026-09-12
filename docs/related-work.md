# 相关工作与新颖性定位

日期：2026-09-06（末次更新 2026-09-09）
状态：ECL 基线已补。检索基于公开文献，非专业专利检索。

本文档的用途有两个：一是把"本项目提出的机制在文献中已有什么"这件事查清楚并写在明面上，二是给出可辩护的贡献声明与**不可**主张的措辞清单。写法参照 `docs/results/2026-08-12-contact-robustness-grid.md` 的 Supported / Unsupported Claims 分节——那是本仓库唯一认真斟酌过claim措辞的地方。

---

## 1. 关键结论：核心机制不是新的

本项目的机制是：**物体与夹爪都用相机光学测量，取其差再叠加到运动学位置上**，使相机→世界外参的平移误差以共模形式抵消：

```
target = gripper_kin + R · (object_cam − gripper_cam)
```

这在视觉伺服文献中对应一个有名字的类别。

### 1.1 ECL（endpoint closed-loop）

> S. Hutchinson, G. D. Hager, P. I. Corke, "A Tutorial on Visual Servo Control," *IEEE Transactions on Robotics and Automation*, 12(5):651–670, 1996.

该综述建立 **EOL（endpoint open-loop）/ ECL（endpoint closed-loop）** 分类。ECL 的定义即"视觉系统同时观测目标与末端执行器上的一点"，结论是 **ECL 对标定明显不敏感**：测量结果不受末端位置误差与目标位置误差影响，只受传感器位姿、传感器标定与传感器测量误差影响。

**这正是本项目所依赖的原理，且是 1996 年就已成教科书内容的分类结论。**

### 1.2 未标定装置下的抓取对齐

> R. Horaud, F. Dornaika, B. Espiau, "Visually Guided Object Grasping," *IEEE Transactions on Robotics and Automation*, 14(4):525–532, 1998.

用同一幅图像中夹爪与物体的对齐完成抓取，使用未标定立体装置，以 3D 射影表示描述"抓取/对齐"关系，**明确不需要相机参数知识**，并给出该类方法可期望的抓取精度分析。

**结论：**"两端同时入镜 → 共模标定误差抵消"**不能作为新颖点主张**。以此申请专利大概率被 1998 年这篇驳回；作为论文的核心贡献会被审稿人一句话打掉。

---

## 2. 与本项目实现的差别（可作为工程贡献的部分）

|  | ECL / Horaud 1998 一类 | 本项目 |
|---|---|---|
| 控制结构 | 迭代图像空间伺服，需在线估计图像雅可比 | **单次测量修正**，随后交给运动规划器 |
| 绝对定位来源 | 无（纯图像空间对齐） | **正运动学**（`gripper_kin`），光学量只提供差分修正 |
| 依赖 | 伺服收敛、雅可比估计（或其在线辨识） | 无雅可比、无迭代 |
| 与规划器关系 | 通常绕开规划器直接做速度控制 | **与 MoveIt 采样规划器直接兼容**（look-and-move） |
| 报告内容 | 精度分析 | **定量容差曲线 + 失败形态分类学** |

未标定视觉伺服的雅可比在线辨识路线亦已成熟：

- K. Hosoda, M. Asada, "Versatile Visual Servoing without Knowledge of True Jacobian," *IROS*, 1994.
- J. A. Piepmeier, G. V. McMurray, H. Lipkin, "A Dynamic Jacobian Estimation Method for Uncalibrated Visual Servoing," *IEEE/ASME AIM*, 1999.
- M. Jägersand 等，基于 Broyden 估计与信赖域的非线性最小二乘路线。

**本项目相对这条线的差别在于完全不做雅可比辨识**——代价是不能纠正旋转类外参误差，收益是零迭代、零在线辨识、可直接嵌进现有 look-and-move + MoveIt 流水线。这个 trade-off 是可以写清楚并量化的。

---

## 3. 更致命的竞争路线：在线重估外参

比"容忍标定误差"更直接的做法是**把外参重新估出来**。这条线近年进展很快，且精度远好于本项目的容差量级：

> T. E. Lee, J. Tremblay, T. To, J. Cheng, T. Mosier, O. Kroemer, D. Fox, S. Birchfield, "Camera-to-Robot Pose Estimation from a Single Image," *ICRA*, 2020. arXiv:1911.09231

单张 RGB 图像检测机器人关键点 + PnP，得到相机→机器人位姿，**精度与传统多帧手眼标定相当**。

> B. C. Sefercik, B. Akgun, "Learning Markerless Robot-Depth Camera Calibration and End-Effector Pose Estimation," *CoRL*, 2022. arXiv:2212.07567

无标记、**基于深度相机**，同时完成外参标定与末端位姿估计，报告"sub-centimeter and sub-deciradian"误差。

> E. Valassakis, K. Dreczkowski, E. Johns, "Learning Eye-in-Hand Camera Calibration from a Single Image," *CoRL*, 2021. arXiv:2111.01245

**这三篇构成对本项目动机的直接挑战**：若外参可以在线估到亚厘米，"把容差从 24mm 提到 240mm"要解决的是什么问题？

可辩护的差异（**需要在实验中支撑，目前仓库里没有证据**）：

- 上述方法**都需要针对该机器人训练的模型**与训练数据；本项目是纯几何的，零训练。
- 本项目**只用深度单通道**，不依赖 RGB 外观、纹理或光照。
- 计算量可忽略，无 GPU 依赖。
- 失效是**两段式**的，分界由视场闭式解预测：夹爪仍在画面内时偏置只损害抓取精度（288mm 处 9/12，失败形态为 `unilateral_contact`，`detector_timeout` 为 0/12）；夹爪离开画面后全部超时（320mm 处 12/12 `detector_timeout`）。**已由 1407 次既有试验的重新分析部分支撑**，见 [失效形态](results/2026-09-12-openarm-degradation-shape.md)。但**不可写成「渐进」**：240mm 与 288mm 在 n=12 下不可区分（Newcombe [−0.041, +0.532]，需 n=27），且该形态只在视场轴上成立——运动学轴（x）上是悬崖（`planning_failure`，可达性二值），同一方法的 `cl_vg` 配置在 288mm 处亦为悬崖（0/12）。

**这几条必须选择性地用实验证明，否则不能写。**

---

## 4. 手眼标定算法（作为"正常做法"的参照）

> R. Y. Tsai, R. K. Lenz, "A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration," *IEEE T-RA*, 5(3):345–358, 1989.
> F. C. Park, B. J. Martin, "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group," *IEEE T-RA*, 10(5):717–721, 1994.
> K. Daniilidis, "Hand-Eye Calibration Using Dual Quaternions," *IJRR*, 18(3):286–298, 1999.

近年对比数据显示这类方法的平移偏差在 **~1mm 量级**。**这是本项目 motivation 的基准线**：需要论证在什么部署条件下标定精度达不到毫米级，或标定本身不可得。

---

## 5. 基准

> F. Bottarel, G. Vezzani, U. Pattacini, L. Natale, "GRASPA 1.0: GRASPA is a Robot Arm graSping Performance benchmArk," *IEEE Robotics and Automation Letters*, 5(2):836–843, 2020. arXiv:2002.05017

抓取基准，显式把**平台的可达性与标定局限**纳入评分，而不是假定平台完美。这是与本项目问题设定最贴近的基准工作，若要做可比性论证应参照它。

---

## 6. 本项目的位置

### 可以主张（Supported）

- 在 **look-and-move + 采样运动规划**架构下，用**一次**相对光学测量即可获得 ECL 级别的标定免疫，无需图像雅可比辨识、无需伺服迭代。**已由 `stage44_baseline` 成对实测支撑**：成功率每格不劣于迭代伺服，手臂移动 1 次 vs 约 12 次。
- **该方案没有内在的容差上限**——它一直有效直到夹爪离开画面，上限即相机视场 `bias_max = cy·z/fy`（本装置 283mm），与 240/288/320/384mm 四点实测吻合。开环基线为 24–32mm（y）/ 32–40mm（x）。
- **失败形态分类**把不同机制分离开：x 向的边界是 IK 可达性（`planning_failure`），y 向的边界是**相机视场**（`detector_timeout`），而 x 的视场边界 `cx·z/fx = 378mm` 远在其运动学边界之后，故 x 永远先撞运动学。
- **精度与上限互不相干**：残差在 ±15mm 内由自定心被动吸收，把残差压到 ±2mm 不改变成功率（`stage40_ceiling`）。
- **修正目标 ≠ 修正路径**：修正后必须回到 hover 再下降，否则对角线扫过物体导致失败（step4 §4）。

### 已补上（2026-09-09）

原「~~优于现有方法~~ —— 尚未与任何已知方法做过对比实验」一条**已部分解除**：ECL 迭代伺服基线已实现并成对测量（`stage44_baseline`，72 次）。可主张的措辞是**在这个探测器上、在任务成功率与移动代价上不劣于迭代伺服**，不是「优于现有方法」这种一般性表述。详见 [ECL 伺服基线对比](results/2026-09-09-openarm-ecl-servo-baseline.md)。§3 的另两条线（DREAM 一类在线重估外参）仍未对比。

### 不可主张（Unsupported）

- ~~"提出了一种利用共模抵消消除手眼标定误差的新方法"~~ —— 见 §1。
- ~~"实现了免标定抓取"~~ —— `rel_offset` 仍是拟合常数。其「依赖标定条件」已查明为**视角相关的几何项**（过读 ≈ 47.8mm·sinθ），θ 可由读数本身算出而不涉外参；但该修正**在拟合区间外主动加害**（288mm 处使成功率从 9/12 降到 0/12），默认关闭。
- ~~"优于现有方法"~~ —— ECL 伺服基线已补（见上），但 §3 的在线重估外参一线仍未对比；且伺服的参数未做穷尽搜索，本实现为位置空间而非严格图像空间 IBVS。
- ~~"鲁棒视觉抓取系统"~~ —— 单一物体、单臂、y≥0 半平面、仅仿真。

---

## 7. 尚需完成的检索

- 专业专利检索（本文档只做了公开文献检索）。已注意到 `WO2000045229A1` "Uncalibrated dynamic mechanical system controller" 与该主题相关，未细读。
- 深度单通道做**夹爪自定位**（非物体定位）的先行工作——这是本项目独有工程组件中最可能有先行者的一块，尚未查到直接对应工作。
- 手眼外参漂移的现实分布测量（若选定"漂移"作为 motivation，需要文献支撑量级）。
