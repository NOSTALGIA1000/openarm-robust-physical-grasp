# 致课程代码作者的许可询问（信稿）

**这封信由仓库所有者自行发送，本文件只是稿子。** 收件人：yhang <1936951548@qq.com>（课程包 `openarm_moveit_config` 在 `package.xml` 中声明的作者与维护者）。

问的是两件、且只有两件事——**上游缺口只有作者能补，其余的已经按 [NOTICE](../NOTICE) 处理好了**：

1. `<license>BSD</license>` 指的是 2-clause 还是 3-clause？
2. 版权行应当写成什么？（署名给谁：作者本人、还是所属院系/学校）

收到答复后：把答复写进 `NOTICE` 与 `THIRD_PARTY.md` §2，并把 §2.1 中关于版本歧义与缺版权行的那段推理降级为脚注。若久无答复，现状可维持——`NOTICE` 已照录 BSD-3 全文（其条件是 BSD-2 的超集）、按上游声明署名，并明说上游未提供版权行、本仓库不代为编造。

---

## 中文稿

> 主题：关于 openarm_moveit_config 课程代码的许可确认
>
> yhang 老师/同学您好：
>
> 我是选修（或接触到）这门课程的学生，课程包 `ros2_ws/src/openarm_moveit_config` 我在其基础上做了一个研究项目：把其中的抓取演示改成有物理接触判定的实验平台，并做了手眼标定误差下的容忍度测量。项目准备以 Apache-2.0 开源，发布前我逐个核查了所有第三方来源的许可，有两处需要向您确认。
>
> 您的包在 `package.xml` 中声明了 `<license>BSD</license>`，但包内没有 LICENSE 文件，源文件里也没有版权行。因此有两点无法确定：
>
> 1. **BSD 的版本**：是 2-clause 还是 3-clause？
> 2. **版权行**：再分发时需要保留版权声明，请问应当写成什么？署名给您个人，还是您所属的院系/实验室/学校？
>
> 我在项目中派生自您这个包的文件，以及派生的程度（按去掉空行与注释后的唯一行计）如下：
>
> | 我的文件 | 源自 | 原文件唯一有效行的存留 |
> |---|---|---|
> | `apple_detector.py` | `launch/apple_detector.py` | 80/87 |
> | `config/openarm_physics.srdf` | `config/openarm.srdf` | 218/257 |
> | `launch/physics_controllers.yaml` | `launch/my_controllers.yaml` | 32/42 |
> | `launch/move_group_physics.launch.py` | `launch/move_group.launch.py` | 11/19 |
> | `launch/run_physics_grasp.launch.py` | `launch/run_simulation_with_detector.launch.py` | 26/71 |
> | `openarm_grasp_physics/physics_picker.py` | `launch/pick_apple.py` | 32/300 |
>
> 在您答复之前，我的仓库中已按 3-clause BSD 的条款照录许可全文并按 `package.xml` 中的声明署名给您，同时如实写明上游未提供版权行、我没有代为拟写。选 3-clause 是因为它的条件是 2-clause 的超集，这样无论哪一种都不会违反；多出的"不得用作背书"那一条约束的是我，不是您。
>
> 如果上面的处置有任何一处您认为不妥，或者您希望我改用别的署名方式、甚至希望我不要再分发某个文件，请直接告诉我，我会照办。
>
> 另外，如果您有兴趣，我很乐意把实验结果和文档发给您——课程那个演示原本是软件吸附的，改成真实接触之后，容忍度的形状和我们当初的直觉很不一样。
>
> 谢谢您。
>
> （姓名 / 联系方式 / 仓库地址）

---

## English version

> Subject: Licence clarification for the openarm_moveit_config course package
>
> Hello,
>
> I built a research project on top of your course package
> `ros2_ws/src/openarm_moveit_config`: I turned its grasping demo into an
> experimental platform with real contact adjudication, and measured grasp
> tolerance under hand-eye calibration error. I intend to release it under
> Apache-2.0, and while auditing third-party provenance before publishing I
> found two things only you can settle.
>
> Your `package.xml` declares `<license>BSD</license>`, but the package ships no
> LICENSE file and the sources carry no copyright line. So:
>
> 1. **Which BSD** — 2-clause or 3-clause?
> 2. **What should the copyright line say?** Redistribution requires retaining a
>    copyright notice. Should it name you personally, or your department, lab or
>    university?
>
> The files I derive from your package, and how much of each original survives
> (unique lines, blanks and comments removed):
>
> | My file | From | Original unique effective lines still present |
> |---|---|---|
> | `apple_detector.py` | `launch/apple_detector.py` | 80/87 |
> | `config/openarm_physics.srdf` | `config/openarm.srdf` | 218/257 |
> | `launch/physics_controllers.yaml` | `launch/my_controllers.yaml` | 32/42 |
> | `launch/move_group_physics.launch.py` | `launch/move_group.launch.py` | 11/19 |
> | `launch/run_physics_grasp.launch.py` | `launch/run_simulation_with_detector.launch.py` | 26/71 |
> | `openarm_grasp_physics/physics_picker.py` | `launch/pick_apple.py` | 32/300 |
>
> Until you reply, my repository reproduces the 3-clause BSD terms in full and
> attributes authorship as your `package.xml` declares it, while stating plainly
> that upstream supplied no copyright line and that I have not invented one. I
> chose the 3-clause text because its conditions are a superset of the
> 2-clause's, so honouring it cannot violate either reading; the extra
> no-endorsement clause binds me, not you.
>
> If any of that is not how you would like it handled — a different attribution,
> or not redistributing a particular file at all — tell me and I will change it.
>
> Separately, I would be glad to send you the results if they interest you. The
> course demo attached the object in software; once the grasp has to survive
> real contact, the shape of the tolerance region is quite unlike what we
> assumed.
>
> Thank you.
>
> (name / contact / repository URL)
