# 壁挂式演示（Wall-Mount Demo）排查与修复报告

日期：2026-09-13

本文档记录壁挂演示（4 个相机传感器盒磁吸挂墙）从"4 个 cycle 大部分失败"到
"端到端 COMPLETE、4/4 焊上"的完整排查与修复过程。方法论沿用
`docs/grasp_fix_report.md`：先取证（关节遥测、物体 ground-truth 位姿轨迹、
磁铁 weld 状态），找到根因再改，不做盲调参数；每项改动附实测依据与可复现的
验证命令。

## 1. 验收标准

4 个 cycle 全部完成；4 个盒子全部焊上墙（weld 成立即 `_magnet_weld_N`
FixedJoint 存在）；偏斜角 < 15°；无 grasp abort。

## 2. 问题清单与解决过程

### 2.0 机械臂高频振荡（排查后排除）

**现象**：怀疑机械臂高频振荡。
**取证**：sim 内 20550 个关节遥测样本，覆盖任务全周期；parked 状态专项采样。
**结论**：仿真内无可测振荡——parked 峰峰抖动 0.00000 rad；最高关节速度
1.5–2.5 rad/s 全部位于正常运动相位。振荡若存在于真机，属真机增益问题，
不在本仿真范围。本条结案。

### 2.1 抓取 V 形夹持 → 盒随机翻滚

**现象**：抓取后盒子随机翻转，交付偏斜 39.3°/90°/51.2°/90°（Run A 实测 4 盒）。
**根因**：riser 角点 approach yaw=atan2(±0.109,±0.109)=±45°；盒为
8×3.5×4 cm、50 g，夹爪最大开口 4 cm 只能跨 3.5 cm 窄边。yaw=±45° 时开口轴
与盒棱成 45°，闭合产生翻转力矩，PhysX 点接触无扭转摩擦，盒必然翻滚。
**修复**：`piper_manipulator` 新增 `grasp_yaw_snap_step`（wall_mount.yaml 置
π）：yaw 绝对值 snap 到最近的 π 倍数再 clamp 到 ±π——角点 0/1 → 0°，角点
2/3 → ±180°，永远平行跨窄边。
**实测依据**：开口 4 cm < 8 cm 边长，π/2 方案（跨长边）物理不可行；π 方案
12/12 目标位姿 IK 全可达（yaw_matrix 实测），π/2 方案 0/8。

### 2.2 翻转盒被偶极子力排斥 / 平贴墙盒永不焊

**现象**：搬运中翻转 90° 平放的盒在 d≈0.07 m 被排斥逃逸；平贴墙的盒在
d=0.046–0.050 m 卡死永不焊（探针逐 tick 力/位姿轨迹实测复现）。
**根因**：纯偶极子-偶极子相互作用在特定姿态-距离组合下为斥力；weld 触发
逻辑与姿态门缺失。
**修复**：磁图（author_magnet_graph.py）增加 attract 捕获 + 距离触发 weld；
`physics_tuning.py` 重标定 `MAGNET_ATTRACT_RANGE=0.10 m`、
`MAGNET_DIPOLE_MOMENT=18.3`（K=2·r⁴/6 校准，保持外缘 2 N 吸力不变，
range 0.10 内逐距离力值实测一致）。磁铁图已重授进 stage USD（4 sensor/target
pairs，已验证）。

### 2.3 歪焊（weld 门不限姿态）

**现象**：旧版图在偏斜 45.8° 时仍焊上。
**修复**：磁图新增 `WELD_MAX_MISALIGN_DEG=15` 姿态门（misalign 超限只吸不
焊，等偶极子扭矩扳正后再焊）。探针实测：24° 偏置 spawn 收敛到 14.1°，门
行为正确。

### 2.4 可达性/工作空间冲突（每个数值都有实测矩阵）

**现象**：修复 yaw 后新的 pregrasp 在 approach_height=0.10 全部不可规划；
retreat 0.15 时外目标 cartesian fraction 0.933/0.967（要求 ≥0.98）。
**修复与依据**（OMPL plan / cartesian fraction 实测矩阵）：

| 参数 | 旧值 | 新值 | 依据 |
|---|---|---|---|
| approach_height | 0.10 | 0.05 | h=0.10 时新 yaw pregrasp 0/4 可规划 |
| place_approach_distance | 0.00 | 0.03 | 释放-捕获距离进入 0.10 attract range |
| retreat_height | 0.15 | 0.12 | 0.15 → fraction 0.933/0.967；0.12 → 1.000 |
| 放置指令点 z（task） | — | +0.06 m | link6 必须高于臂下工作空间边界 ~-0.05 m |

### 2.5 place 阶段 move_group 段错误（SIGSEGV，E2E 卡死根因）

**现象**：连续两次 E2E 均在 place[1] MOVE_ARM_TO_PREPLACE 卡死 145 s 后任务
中止，move_group exit code -11。
**取证**（两次完整日志，时间线一致）：
1. preplace 的 plan+execute 实际耗时 ~35 s（5 s OMPL 规划 + 慢速轨迹执行，
   stage 上关节状态率仅 ~50 Hz，轨迹实时放慢）；
2. piper 客户端硬编码 result 超时 30 s（`piper_manipulator_node.cpp`）→
   1902 s 误判超时，重试发第二个 goal；
3. 1907 s 第二个 goal 到达时第一个轨迹仍在执行 → move_group
   "Cannot push a new trajectory while another is being executed" →
   trajectory initialization failed → **进程 SIGSEGV** → 后续 goal 全部
   "goal send timed out"，任务 abort。
**修复**（`piper_manipulator`）：
1. result 超时参数化 `move_action_result_timeout`（默认 120 s，实测依据写入
   注释）；
2. 重试循环在 attempt>1 时先 `async_cancel_all_goals()` 并等待 2 s，取消
   在途 goal 再重发，杜绝并发 goal 触发 move_group 崩溃路径。
修复后容器内 colcon build 通过，同一 E2E 完整跑通。

### 2.6 放置姿态重定向（place_reorient）

**现象**：搬运中盒姿态不确定，平放盒在 2.2 被拒焊。
**修复**：新增 `place_reorient`（wall_mount.yaml=true）：place 阶段不沿用抓取
姿态，改用**绝对 TCP 目标帧**——opening=(0,-1,0)，approach_dir 由任务给定，
tilt = π/2 − grasp_pitch + 0.415（45° 下倾），fingertip 轴
(0.707,0,-0.707)，link6 = place_arm − 0.13503·approach_dir。
**实测依据**：理想 tilt=90°−grasp_pitch 姿态精确但 IK 0/30；29° 只外目标可
达；37° 部分；45° 全部 12/12 可达且释放偏斜 ~24° 可被偶极子扭矩收敛。

## 3. 最终结果

| 验证项 | 结果 |
|---|---|
| 生产语义链式 sweep（4 cycle × 6 阶段，起始状态逐腿串联，墙+riser 作 CollisionObject） | 24/24 PASS（/tmp/ik_sweep2.py，PLACE z=-0.06；z=-0.066 时内目标 fraction 0.917 失败） |
| 磁铁探针 ×3（in-range 捕获 / out-range 静止 / 24° 偏置对齐） | 3/3 PASS（捕获位移 0.0723 m、焊上、末距 0.0154 m） |
| 端到端 headless E2E | **wall-mount task COMPLETE；4/4 welded**（d=0.0142–0.0154 m，misalign 14.4–15.0°，y 偏差 ≤2 mm），0 ERROR，0 unweld |
| 可视化 GUI E2E（display :1，run_wall_stage_gui.py） | 任务 COMPLETE、0 ERROR；3/4 welded（sensor 3 搬运摆动中滑脱飞出，落点正确但 misalign=15.6°，超 15° 门 0.6° 被拒焊，靠 0.245 N 磁力贴墙未焊） |

## 4. 修改文件清单

- `src/kotek_isaac_stage/kotek_isaac_stage/physics_tuning.py`
  （range 0.10 / moment 18.3 / WELD_MAX_MISALIGN_DEG=15）
- `src/kotek_isaac_stage/kotek_isaac_stage/author_magnet_graph.py`
  （weld 姿态门接线）
- `src/kotek_isaac_stage/usd/kotek_scout_piper_wall_demo.usd`
  （磁铁图重授，4 pairs）
- `src/kotek_manipulation/include/kotek_manipulation/piper_manipulator.hpp`
  （grasp_yaw_snap_step / place_reorient / move_action_result_timeout）
- `src/kotek_manipulation/src/piper_manipulator_node.cpp`
  （yaw snap、绝对目标帧 place、超时参数化、重试前 cancel）
- `src/kotek_bringup/config/wall_mount.yaml`（上述参数落地）
- `src/kotek_wall_mount_task/kotek_wall_mount_task/wall_mount_task.py`
  （`_WALL_Z` 指令点上移 6 cm）

## 5. 复现命令

```bash
# Isaac（headless 验收；GUI 可视化把脚本换成 run_wall_stage_gui.py 并加 DISPLAY=:1）
cd /home/trs/kotek_ws
ROS_DOMAIN_ID=77 KOTEK_WITH_ROS=1 ./run_isaac.sh \
  src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage.py --duration 2400

# ROS 栈（等 "### physics playing" 与 "piper_manipulator ready" 后再等 45 s）
docker compose -f docker/compose.yaml run --rm -e ROS_DOMAIN_ID=77 kotek bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install \
   --packages-skip scout_nav2_pkg && source install/setup.bash && \
   ros2 launch kotek_bringup kotek_wall_mount_demo.launch.py'

# 任务（容器内）；焊接证据在 Isaac 侧日志 grep "welded sensor"
ros2 run kotek_wall_mount_task wall_mount_task
```

## 6. 遗留风险与后续建议

1. **15° 门处于自然平衡带上**：偶极子对齐后的平衡偏斜稳定在 14–15.6°，
   余量极小（GUI 运行中 sensor 3 以 15.6° 被门拒焊证实）。建议优化 release
   姿态/捕获距离，让扭矩有更长时间把偏斜扳到 15° 以下，而不是放宽门限。
2. **搬运夹持裕度有限**：PhysX 点接触无扭转摩擦，GUI 低帧率（~44 Hz）下
   摆动惯量曾把盒甩脱。建议给任务/placer 增加"盒在爪中"校验（当前盒丢了
   任务仍报 COMPLETE）。
3. **GUI 运行与 headless 的差异**：渲染负载降低关节状态率、改变时序，
   正式验收走 headless runner（已 4/4 全焊）。
4. OMPL 规划存在随机性（sweep 中 preplace[2] 曾单次失败），重试机制已兜底。
