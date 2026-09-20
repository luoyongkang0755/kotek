# Kotek 下一步工作 Prompt（2026-09-14 → 09-27）

> 用法：把下面 "ENGLISH PROMPT" 整段贴给编码 AI。所有数字均来自 50 遍批量实测（docs/wall_mount_e2e_50runs/README.md 与 docs/wall_mount_fix_report.md），AI 可在仓库中自查验证。

---

## ENGLISH PROMPT

```text
You are working in the kotek_ws repository (ROS 2 Jazzy + MoveIt + Isaac Sim 6.0.1 aarch64, Docker).
Context: the wall-mount demo pipeline is now stable — 50/50 strictly-serial E2E runs completed with
0 aborts and 0 SIGSEGV (batch harness: run_e2e_50.sh). Per-box weld success is 134/200 = 67.0%;
6/50 runs achieved a full 4/4 (12%). All 66 unwelded boxes fall into three cleanly separated classes
(see docs/wall_mount_e2e_50runs/README.md):

- slip_near_wall   33x (50%) — box dropped at the wall during place, landed flat, out of the 0.10 m
  attract range; heavily concentrated on the OUTER targets (s4×18, s1×9, s2×0). The long preplace
  swing inertia pries the box out of the fingers (PhysX point contacts have no torsional friction).
- gate_borderline  31x (47%) — box captured at the wall (d≈0.015 m) but equilibrium misalign is
  15.1–22.1° (median 15.5°), exceeding the current WELD_MAX_MISALIGN_DEG = 15 gate. Welded boxes
  sit at 13.6–15.0°.
- gate_jammed       2x (3%)  — box pressed onto the wall at 29.5°/43.5°, geometrically stuck;
  dipole torque τ≈1e-4 N·m is too weak to correct it.

Measured facts you must respect:
- Equilibrium band tops at 22.1°; jams start at 29.5°. A 23° gate flips all 31 borderline rejects
  while still blocking the 45.8°-class skewed welds that the gate was created for.
- Today a lost box still reports task COMPLETE (no in-hand verification before release).
- Current per-target weld rates: s1 72% / s2 74% / s3 60% / s4 62%.

TASKS (in priority order)

Task 1 — QUICK WIN: widen the weld gate.
  Change WELD_MAX_MISALIGN_DEG from 15 to 23. Add a comment citing the measured basis
  (equilibrium band ≤22.1°, jam onset 29.5°, 50-run batch). Nothing else changes.
  Expected: per-box weld rate 67% → ~82%.

Task 2 — MAIN EFFORT: outer-target anti-slip (s1 / s4).
  a) Slower carry velocity/acceleration scaling specifically on the LONG preplace swing segment
     (the segment that pries boxes out of the gripper). Do not slow down short moves — keep the
     376–385 s per-run budget roughly intact.
  b) Add a box-in-hand check before release (e.g., contact/joint-state based grasp verification).
     If the box is lost, the task must NOT report COMPLETE — it should re-grasp or fail loudly.
  Expected (combined with Task 1): per-box ~95%+, full 4/4 rate 12% → ~80%+.

Task 3 — LONGER TERM: release-pose optimization.
  Push the natural equilibrium band down (release skew ~24° currently converges to 13.6–22.1°)
  by optimizing the release pose, instead of widening the weld gate further.

WORKING AGREEMENT (mandatory)
- Every change lands with measured evidence and is re-checked against the same 50-run batch
  (run_e2e_50.sh) before completion: report per-box weld rate, full-4/4 rate, per-target rates,
  and the failure-class histogram, compared against the 67% / 12% baseline.
- Do not regress the existing fixes: grasp_yaw_snap_step = π, MAGNET_ATTRACT_RANGE = 0.10 m,
  MAGNET_DIPOLE_MOMENT = 18.3, move_action_result_timeout = 120 s, drive damping ratio = 0.1,
  place reorient absolute TCP frame (opening (0,−1,0), tilt 45°).
- Run Task 1 and validate it FIRST; Task 2 builds on the new gate. Keep changes in small,
  separately-testable commits.

Deliverables: code changes + a short report doc (docs/) with before/after batch metrics.
```

---

## 中文对照（任务要点）

1. **Task 1（见效快）**：`WELD_MAX_MISALIGN_DEG` 15° → 23°。依据：平衡带上限 22.1°、卡死下限 29.5°，可救回全部 31 例边缘拒焊，仍挡得住 45.8° 级歪焊。预期单盒焊接率 67% → ~82%。
2. **Task 2（主要工作）**：外侧目标防掉落（s1/s4）——
   - a) 只在**长 preplace 摆动段**降低搬运速度/加速度缩放，别拖慢整体节奏（单遍 376–385 s 预算尽量保持）；
   - b) 释放前增加"盒在爪中"校验（接触/关节状态判定），盒子丢失时不许再报 COMPLETE，要重新抓取或明确报错。
   - 与 Task 1 叠加预期：单盒 ~95%+，4/4 全焊率 12% → ~80%+。
3. **Task 3（长期）**：优化释放姿态，把自然平衡带整体下压（当前释放偏斜 ~24°，收敛到 13.6–22.1°），而不是继续放宽门限。

**工作约定（必须写进 prompt 的硬约束）：**
- 每项改动都用同一套 50 遍批量（`run_e2e_50.sh`）复验，报告对比基线（67% / 12%）的前后指标；
- 不得回归已有修复：yaw 吸附 π、磁吸参数 0.10 m / 18.3、超时 120 s、阻尼比 0.1、放置绝对 TCP 帧；
- 先单独验证 Task 1，再做 Task 2；小步提交、分别可测。

---

## 真机线（提示，不在编码 prompt 内）

实体机器人线（首次监督运行、deadman/锁存急停安全机制、真机 vs 仿真 67% 基线对比）属于实验安排，不需要写进编码 prompt——等仿真侧 Task 1+2 达标后再启动即可。
