# [VBC-PHYSICAL-GRIPPER] Go2 + Piper VBC 训练说明

## 这次新增了什么

本次保留旧任务不变，并新增两个独立分支：

- `GO2-PIPER-VBC-Teacher`：用物体状态训练 privileged 高层 teacher，先验证物理抓取闭环。
- `GO2-PIPER-VBC-Student`：在同一个物理场景和同一个冻结 WBC 上，用双相机 RGB-D 通过 RSL-RL DistillationRunner 学视觉 student。

```text
特权物体状态
      ↓
10 维高层 VBC teacher
      ↓  [vx, vy, wz, Δee_xyz, Δee_rpy, gripper]
冻结的 LeggedManipLab 18 维 WBC policy
      ↓
Go2 12 个腿关节 + Piper 6 个臂关节
      └── 物理 gripper PD → joint7 + joint8
```

这里的 student 也不是把 RGB-D 图像拼入原来的 210 维 WBC policy。低层仍然是原来的 210-D observation/18-D action；图像只进入高层 student，输出高层命令，再由高层 action term 调用冻结 WBC 和物理夹爪。

## 论文原方法与当前实现的关系

原论文的真实训练链路是三阶段，而不是一个 RGB 网络直接回归 19 个关节：

```text
低层 RL：目标末端/底盘命令 → 全身跟踪控制器
                         ↓ 固定
特权 state teacher：物体点云/位姿 + 本体状态 → 高层动作
                         ↓ 在线 teacher-action imitation / DAgger
视觉 student：双相机 mask/分割深度 + 本体状态 + 历史动作 → 同一个高层动作
```

论文的高层动作是 9 维：末端位姿增量 6 维、底盘速度 2 维、夹爪开合 1 维。六个臂关节不是 PPO 的原始 action，而是末端位姿命令通过伪逆 Jacobian IK 得到；12 个腿关节由低层 RL 输出。夹爪是独立的二值命令和执行器。论文用动态物体的实际接触与“物体抬高并保持”判断抓取成功，没有使用 attach/weld。

当前 Go2+Piper 适配器保留 `vx, vy, wz` 三个底盘量，所以高层动作是 10 维：`[vx, vy, wz, Δee_xyz, Δee_rpy, gripper]`。这是为了适配现有 Go2 的底盘接口，不是论文原始 9 维定义。当前 student 的结构和训练顺序对齐论文，但输入先采用仿真可直接获得的 RGB-D；论文真实部署还使用目标分割 mask/segmented depth 和 TrackingSAM，因此当前不能宣称已经完成论文级视觉泛化。

## 新增文件

所有新增文件均带有 `[VBC-NEW]` 标记：

| 文件 | 作用 |
|---|---|
| `.../assets/go2_piper/go2_piper_articulation_cfg.py` | 新的 Isaac URDF 资产配置；旧 USD 仍由 `GO2_PIPER_CFG` 使用 |
| `.../mdp/vbc_actions.py` | 10 维高层动作到冻结 18 维 WBC + 两个物理夹爪关节 |
| `.../mdp/vbc_mdp.py` | 方块距离、夹爪受力、抬升高度、保持窗口和课程 |
| `.../config/go2_piper/vbc_env_cfg.py` | 平面、桌面、动态方块、物理夹爪 teacher observation、奖励和课程 |
| `.../config/go2_piper/agents/rsl_rl_ppo_cfg.py` | 独立 teacher PPO 和 CNN visual-student distillation 配置 |
| `scripts/prepare_go2_piper_vbc_urdf.py` | 从 Gazebo URDF 生成 Isaac 专用 URDF，保留 joint7/joint8 和碰撞体 |
| `scripts/verify_go2_piper_usd_gripper.py` | 转换后检查 USD 中是否为两个 `PhysicsPrismaticJoint` |
| `scripts/rsl_rl/train.py` | 支持 `--teacher_checkpoint` 加载已训练 teacher |
| `docs/VBC_GO2_PIPER_IMPLEMENTATION_CN.md` | 本说明 |

## 修改文件

修改仅增加 VBC 注册/导出/配置，不改旧任务的行为：

| 文件 | 标注位置 | 改动 |
|---|---|---|
| `.../mdp/__init__.py` | 文件末尾 `[VBC-NEW]` | 导出 VBC action/MDP term |
| `.../config/go2_piper/__init__.py` | 文件末尾 `[VBC-NEW]`/`[VBC-VISION-STUDENT]` | 增加独立 teacher/student 任务 ID |
| `.../config/go2_piper/agents/rsl_rl_ppo_cfg.py` | 文件末尾 `[VBC-NEW]`/`[VBC-VISION-STUDENT]` | 增加独立 teacher PPO 和 student distillation 配置 |
| `docker/docker_isaaclab.sh` | VBC 资源挂载/准备步骤 | 只读挂载 Go2Arm，用于生成 Isaac 专用 URDF |

原有 `GO2-PIPER-Flat`、`GO2-PIPER-WBC`、`GO2-PIPER-Stairs` 注册名、旧 policy 文件和旧 action/observation contract 没有被替换。

## 启动训练

在 Isaac Sim/Isaac Lab 容器中执行：

```bash
cd /workspace/LeggedManip_Lab

export LEGGEDMANIP_WBC_POLICY=/workspace/LeggedManip_Lab/mujoco/deploy/policy/go2_piper/wbc/policy.pt

python3 scripts/rsl_rl/train.py \
  --task GO2-PIPER-VBC-Teacher \
  --num_envs 2048 \
  --headless \
  --max_iterations 10000
```

上面的 `export` 要在容器内部执行才会生效；如果是在宿主机调用仓库已有的 `isaaclab_train` helper，容器内默认路径已经是 `/workspace/LeggedManip_Lab/mujoco/deploy/policy/go2_piper/wbc/policy.pt`，不需要依赖宿主机的 export。

仓库的 Docker 脚本会把源码复制到容器内的 `/tmp/LeggedManip_Lab` 并做 editable install。源码更新后，第一次运行前要清掉旧安装标记，保证新增 VBC 文件同步进去：

```bash
docker exec isaac-sim rm -f /tmp/LeggedManip_Lab/.installed
```

如果容器还没有启动，先执行仓库已有的 `isaaclab_start`/`isaaclab_setup`。setup 会自动生成 `go2_piper_vbc.urdf`；如果你不是通过该脚本启动容器，也可以手动执行：

```bash
python3 scripts/prepare_go2_piper_vbc_urdf.py \
  --input /workspace/Go2Arm_sim2sim/ros2_ws/src/go2_piper_description/urdf/go2_piper_ros2.urdf \
  --output /tmp/go2_piper_vbc.urdf \
  --mesh-root /workspace/Go2Arm_sim2sim/ros2_ws/src/go2_piper_description/legacy/meshes
cp /tmp/go2_piper_vbc.urdf \
  /tmp/LeggedManip_Lab/LeggedManip_Lab/assets/go2_piper/go2_piper_vbc.urdf
```

如果使用仓库已有的容器快捷命令：

```bash
cd /workspace/LeggedManip_Lab
isaaclab_train GO2-PIPER-VBC-Teacher true 2048 10000
```

如果策略文件不在默认路径，必须通过 `LEGGEDMANIP_WBC_POLICY` 指向绝对路径。

建议先用小规模 smoke test 验证环境能构造、WBC 能加载，再开长训练：

```bash
isaaclab_train GO2-PIPER-VBC-Teacher true 16 10
```

确认没有维度、资产或物理初始化错误后，再执行 `2048` 环境、`10000` iteration 的正式训练。

## 当前物理夹爪实现

方块是带质量、重力、碰撞和摩擦的 `RigidObject`，不是 Gazebo fixed joint/attach，因此它不会被训练环境强行粘到夹爪上。

这里需要区分三个控制接口，不能把它们混为“模型只有 18 个关节”:

1. Gazebo/URDF 资产实际上包含 `joint7` 和 `joint8` 两个 prismatic 夹爪指关节，指尖也有碰撞几何；`controllers.yaml` 和现有 Gazebo gripper controller 已经能够单独控制它们。
2. 当前 IsaacLab `go2_piper.usd`/`go2_piper_articulation_cfg.py` 的可控 actuator 配置只覆盖 12 个腿关节和 `joint1`--`joint6`，当前冻结的 MuJoCo WBC checkpoint 也是 `210-D -> 18-D`，不输出 `joint7`/`joint8`。
3. 当前 IsaacLab VBC 资产不再使用旧固定夹爪 USD，而是由含 `joint7/joint8` 的 Gazebo URDF 生成独立的 Isaac URDF/USD；两个关节配置了物理 PD actuator。
4. 高层最后一个标量被二值化并镜像成 `(joint7, joint8)` 的 `(open, open)` 或 `(close, close)` 目标；冻结 WBC 仍只接收原来的 18 个腿/臂目标。
5. 方块是独立动态刚体。抓取奖励要求手指 PhysX net force、闭合命令、末端接近、物体离桌抬升和保持窗口同时满足；没有 attach/weld，也没有直接修改方块位姿。

注意：物理夹爪代码、USD 转换和 PhysX 初始化已经完成 smoke test；这只证明接线正确，不代表长时间 PPO 已经学会稳定抓取。

## 训练课程

1. 桌面固定，方块初始在末端工作空间附近；方块 x/y 只做 ±0.12 m 随机化。
2. 机器人 reset 时保留 WBC 标称腿姿态和 Piper 零位，避免初始姿态变化掩盖高层命令学习。
3. 当 `object_reach` 平均表现达到阈值时，方块 x/y 随机范围每次增加 0.025 m，最大约 ±0.30 m。
4. 奖励同时约束末端跟踪、方块距离、物理夹爪接触、抬升进度、保持成功、机身高度/水平、竖直速度、角速度、力矩、动作变化和足端滑移。

## 重要说明

`VBC-Teacher` 不是最终的视觉部署策略，但已经包含物理夹爪动作和抓取判据。`VBC-Student` 的代码和 1 环境 smoke test 已经接入；真正的大规模 teacher 收敛、student 泛化和部署还需要继续训练评测。当前 student 包含：

- 机身 RGB-D + 腕部 RGB-D sensor；
- 4 帧历史图像，双相机 RGB-D 合并为 `32×84×84`（每个相机 RGB-D=4 通道）；
- `policy=61` 维本体输入、`images=32×84×84`、`teacher=65` 维特权输入；
- CNN student 输出 10 维高层动作，MLP teacher 输出同一 10 维动作；
- RSL-RL `DistillationRunner` 的在线 teacher-action imitation，使用 MSE，而不是直接把图片输入旧 WBC actor。

当前第一版仍有三处与论文不完全相同：没有 PointNet++ 的 1024 维物体形状特征；图像先用 RGB-D、尚未接入论文真实部署的 TrackingSAM mask/segmented depth；student 还没有接入 ROS2/Gazebo 的 CNN 部署适配器。因此目前是“论文结构对齐的 Go2+Piper 第一版”，不是论文的逐项复现。

## 训练顺序与命令

先训练 privileged teacher，确保动态方块能靠真实指关节闭合、抬升并保持：

```bash
source docker/docker_isaaclab.sh
isaaclab_run "python3 scripts/rsl_rl/train.py \
  --task GO2-PIPER-VBC-Teacher --num_envs 2048 --headless \
  --max_iterations 10000"
```

再将 teacher checkpoint 传给视觉 student：

```bash
source docker/docker_isaaclab.sh
isaaclab_run "python3 scripts/rsl_rl/train.py \
  --task GO2-PIPER-VBC-Student --num_envs 128 --headless --enable_cameras \
  --max_iterations 10000 \
  --teacher_checkpoint \
  /workspace/LeggedManip_Lab/logs/rsl_rl/go2_piper_vbc_teacher/<run>/model_<N>.pt"
```

两台相机在这个 student 里只负责视觉抓取输入，不承担导航或楼梯规划；高层虽然保留底盘速度输出，但当前分支仍是平面桌面抓取任务。

## 本轮验证状态

- 已用仓库中的 `mujoco/deploy/policy/go2_piper/wbc/policy.pt` 做接口检查：`210-D input -> 18-D output`，输出为有限值。
- 新增 Python 文件已通过 AST 语法检查，注册名和旧任务入口均保留。
- 已在宿主机用纯 Python 检查生成器：输出保留 `joint7/joint8`、Link7/8 碰撞和 `end_effector` helper frame，并移除 Gazebo/ROS/sensor 插件。
- 新旧资产隔离：旧 `GO2_PIPER_CFG`/旧任务仍指向原 `go2_piper.usd`；只有 VBC teacher 指向新 URDF。
- VBC teacher 已在 Isaac Sim 5.1.0 容器中完成 1 环境/1 iteration 的真实 USD→PhysX smoke test；动作形状为 10，teacher observation 为 65，物理夹爪奖励项已加载。
- 转换后的 USD 已由 Isaac Sim schema 检查确认两个 active `PhysicsPrismaticJoint`：`/go2_piper/joints/joint7` 和 `/go2_piper/joints/joint8`，并应用 `PhysxJointAPI`、线性 `PhysicsDriveAPI`。
- VBC student 已完成 1 环境/1 iteration、双相机渲染和 teacher checkpoint 加载 smoke test：`policy=61`、`images=(32,84,84)`、`teacher=65`、student CNN 输入 32 通道，蒸馏迭代正常结束。
- 上述 smoke test 只证明接线、资产和运行时初始化正确，不代表物理抓取成功率或视觉泛化已经收敛；正式训练仍需在已接受 Isaac Sim 许可的容器中执行。转换后可运行：

```bash
python3 scripts/verify_go2_piper_usd_gripper.py \
  /tmp/LeggedManip_Lab/LeggedManip_Lab/assets/go2_piper/configuration/go2_piper_vbc.usd
```
