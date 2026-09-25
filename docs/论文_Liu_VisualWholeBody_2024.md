# Visual Whole-Body Control for Legged Loco-Manipulation：论文精读

## 基本信息

- **题目**：Visual Whole-Body Control for Legged Loco-Manipulation
- **作者**：Minghuan Liu 等
- **版本**：CoRL 2024；公开论文版本为 arXiv:2403.16967
- **论文**：[arXiv 页面](https://arxiv.org/abs/2403.16967)
- **项目页**：[Whole-body B1](https://wholebody-b1.github.io/)
- **官方代码**：[visual_wholebody](https://github.com/Ericonaldo/visual_wholebody)

## 5C 快速筛选

### Category

四足机器人移动操作、视觉伺服、全身控制、强化学习和 sim-to-real。系统使用 Unitree B1、Z1 六自由度机械臂和一自由度夹爪。

### Context

目标不是让一个 PPO 网络直接输出全部硬件关节，而是把“高层任务决策”和“低层全身稳定控制”分开。低层先学会在不同地形和扰动下跟踪底盘速度与末端位姿命令；高层再学会围绕物体完成接近、闭合夹爪和抬升。

### Contribution

1. 用 RL 训练可在粗糙地形上工作的低层 whole-body goal-reaching policy。
2. 在冻结低层的基础上，用物体点云/位姿和本体状态训练 privileged state-based teacher。
3. 用在线 imitation/DAgger 把 teacher 蒸馏成只看视觉输入的 student，而不是直接从稀疏抓取成功信号训练视觉 PPO。
4. 高层动作包括末端位姿增量、底盘速度和二值夹爪状态；机械臂位姿命令经伪逆 Jacobian IK 转为六个臂关节目标。

### Correctness

论文没有把夹爪物理连接到物体上。仿真中使用动态物体、真实夹爪关节和碰撞，抓取是否成功用“物体被抬升并保持”判断；论文文字没有声称使用 attach/weld。官方配置中还有 lifted threshold 和 hold steps，这个判据比单纯末端接近更接近真实抓取。

### Continuity

论文原文和官方实现把训练拆成可复用的三段：低层 WBC checkpoint → state teacher → visual student。对 Go2+Piper 项目，最安全的迁移也是先修复夹爪物理资产并训练 teacher，再接相机和 student。

## 摘要拆解

论文要解决的是：四足底盘自身会移动、俯仰和上下起伏，机械臂又要同时完成末端跟踪，因此固定底盘上的机械臂规划不够。作者选择用 learned whole-body controller 处理腿和臂之间的耦合，再用视觉高层策略完成抓取。

## 方法详解

### 1. 低层 RL whole-body goal reaching

低层的输入是本体状态、历史动作、外部扰动 latent 和高层命令。命令包含：

- 末端目标位置和方向；
- 底盘线速度和偏航角速度。

低层 PPO 的直接动作是 **12 个腿关节目标位置**。机械臂不由 PPO 直接输出六个原始关节动作，而是把末端位姿误差送入伪逆 Jacobian IK，得到六个臂关节增量，然后由关节 PD 执行。这样低层策略负责“腿如何配合身体稳定、身体如何配合末端”，而不是负责机械臂的解析逆运动学本身。

低层训练随机化地形、摩擦、质量、质心、执行器等因素，并通过 curriculum 扩展末端目标范围。因此“全身”指控制耦合和状态闭环，不等于 PPO 输出向量必须是 19 维。

### 2. Privileged state-based teacher

teacher 的高层动作是论文定义的 9 维：

```text
[Δ末端位置(3), Δ末端方向(3), 底盘线速度/偏航速度(2), 夹爪开合(1)]
```

在当前 Go2+Piper 的低层接口里，底盘命令已有 `vx, vy, wz` 三个量，所以适配后会是 10 维：3 个底盘量 + 6 个末端位姿增量 + 1 个夹爪开合量。这是接口适配，不应误称为论文原始 9 维。

teacher 看到的是特权状态：物体形状特征、物体相对机械臂基座的位姿和本体状态。teacher 的 critic 还可以看到更多仿真信息。高层每次动作会保持若干低层控制周期，形成时间尺度分离。

抓取奖励按阶段启用：先鼓励靠近，再鼓励抬升，最后给成功奖励；不能只用末端到物体的距离奖励，否则策略可能学成“碰一下/靠近一下”而不是夹住并抬起来。

### 3. Visual student

student 并不替换低层 WBC。它只替换高层 teacher 的输入：

```text
两路相机的物体 mask + 分割深度 + 本体状态 + 上一步高层动作
                         ↓
                  高层视觉 student
                         ↓
      与 teacher 相同的高层动作（位姿增量、底盘速度、夹爪开合）
                         ↓
          固定低层 WBC + IK + 夹爪执行器
```

论文/官方实现的视觉输入不是“未经处理的整幅 RGB 直接端到端输出关节”。两台 RealSense 分别提供机身前视和靠近末端的视角；系统使用物体 segmentation mask 和 segmented depth，并堆叠最近若干帧，再由 CNN 编码。实际实验中目标掩码的初始化还需要人工在一次 reset 时标注，之后通过 TrackingSAM/AOT 跨帧、跨视角跟踪。因此它是视觉高层控制 student，但不是完全不带目标初始化的开放世界检测器。

## 物理夹爪结论

硬件自由度是 12 腿 + 6 臂 + 1 个夹爪自由度。论文的“一个夹爪命令”在双指机构上可以展开为两个对称的 prismatic joint target；这并不意味着 USD 中必须只有一个 joint。对当前 Piper，`joint7` 和 `joint8` 是两个相反方向的平移关节，应该由同一个二值开/合命令镜像控制。

成功判定至少需要同时满足：夹爪处于闭合状态、手指发生物理接触/受力、物体已经离开桌面并在保持窗口内没有掉落。不能使用 `attach`、固定约束或只检查末端距离。

## 与 Go2+Piper 项目的对应关系

| 论文结构 | Go2+Piper 实现 |
|---|---|
| B1 + Z1 | Go2 + Piper |
| 12 腿低层 | 现有 18 维 WBC 的前 12 维腿动作 |
| 6 臂 IK/PD | 现有 WBC 的 6 维末端命令和关节 PD |
| 二值 gripper | `joint7/joint8` 对称目标 |
| privileged teacher | IsaacLab 中先使用 cube 真值位姿 |
| visual student | 下一阶段加入机身/腕部相机，输出同一高层动作 |

## 局限与复现风险

1. 当前 IsaacLab 原 USD 并没有激活 `joint7/joint8`，所以不能直接在旧 USD 上加 actuator。
2. 论文的低层 policy checkpoint 与 Go2+Piper 的关节顺序、末端 frame、PD 参数必须逐项校验。
3. 视觉 student 的 mask 初始化和相机延迟是实验系统的一部分；只接 RGB/D 不等于复现论文。
4. 抓取成功必须用动态物体的抬升/保持验证，不能用软件 attach 代替。

## 复现实施顺序

1. 生成只供 IsaacLab 使用的 URDF，并确认 USD 中有可驱动的 `joint7/joint8` 和手指碰撞体。
2. 固定已有 18 维 WBC，增加一个高层夹爪开/合动作，由 action term 镜像到两个 prismatic joints。
3. 用物理接触、物体抬升和保持窗口定义抓取成功，先训练特权 teacher。
4. teacher 稳定后再加机身 RGB-D 和腕部 RGB-D，做 mask/depth 预处理与 DAgger student。
5. 最后才加入移动、复杂地形和楼梯课程。

