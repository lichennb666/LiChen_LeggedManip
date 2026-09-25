# 任务计划：Go2Arm 课程实验文档与环境脚本

## Goal
交付 Docker 版与原生 ROS 2 Humble 版课程实验指导书，以及统一环境检查、安装和编译脚本。

## Next Step
实现统一环境脚本并生成两份 DOCX。

## Current Phase
Phase 3

## Phases

### Phase 1：需求与发现
- [x] 理解用户意图
- [x] 确认约束和验收条件
- [x] 将发现写入 findings.md
- **Status:** complete

### Phase 2：规划与结构
- [x] 确定方案
- [x] 记录决策及理由
- **Status:** complete

### Phase 3：实现
- [ ] 按计划执行
- [ ] 增量验证
- **Status:** in_progress

### Phase 4：测试与验证
- [ ] 验证全部需求
- [ ] 将结果写入 progress.md
- **Status:** pending

### Phase 5：交付
- [ ] 复核输出
- [ ] 向用户交付
- **Status:** pending

## Decisions Made
| 决策 | 理由 |
|----------|-----------|
| 两份 DOCX + 一份统一脚本 | 满足 Docker 与原生 Humble 两类课程环境，同时避免重复维护脚本 |
| compact_reference_guide + editorial_cover | 文档是长篇操作手册，需要紧凑查阅结构和清晰封面 |
| 脚本默认 check，install 显式执行 | 防止课程脚本未经确认修改系统 |
| 原生模式创建 /workspace 兼容链接 | 当前策略、地图和配置使用固定 /workspace/Go2Arm_sim2sim 路径 |

## Errors Encountered
| 错误 | 解决方案 |
|-------|------------|
| Dockerfile 依赖未在仓库定义的基础镜像 | 脚本检测基础/最终镜像，文档要求教师分发镜像 tar 或预先构建基础镜像 |
