#!/usr/bin/env bash
# ============================================================================
# run_e2e_50.sh -- run the wall-mount E2E verification N times, record results
# ============================================================================
# One iteration = fresh Isaac (headless run_wall_stage.py) + fresh container
# stack + 45s warm-up + wall_mount_task, per the operational convention that
# re-running needs fresh state on BOTH sides (AGENTS.md / README 7.2).
#
# Per run we record: task result (COMPLETE/ABORTED/TIMEOUT/INFRA), weld count
# (0-4), per-sensor weld misalignment, abort stage + error text, wall time,
# and a coarse failure classification for the aggregate report.
#
# Resumable: iterations already present in results.csv are skipped, so the
# batch can be restarted after an interruption.
#
# Usage: ./docs/wall_mount_e2e_50runs/run_e2e_50.sh [runs] [outdir]
# ============================================================================
set -uo pipefail
cd /home/trs/kotek_ws

RUNS="${1:-50}"
OUT="${2:-/tmp/e2e50}"
mkdir -p "$OUT/fail_logs"
CSV="$OUT/results.csv"
[ -f "$CSV" ] || echo "run,result,welds,misalign_deg,abort_stage,error,duration_s,classification" > "$CSV"

pad() { printf '%02d' "$1"; }

teardown() {
  pkill -f 'run_wall_stage\.py' 2>/dev/null
  docker rm -f $(docker ps -aq --filter name=kotek-run) >/dev/null 2>&1
  sleep 3
}

record() { # result welds misalign abort_stage error classification
  echo "$i,$1,$2,$3,$4,$5,$(( $(date +%s) - t0 )),$6" >> "$CSV"
}

for i in $(seq 1 "$RUNS"); do
  if grep -q "^$i," "$CSV" 2>/dev/null; then
    echo "[run $i] already recorded, skipping"
    continue
  fi
  t0=$(date +%s)
  SLOG="$OUT/run_$(pad $i)_stage.log"
  TLOG="$OUT/run_$(pad $i)_task.log"
  KLOG="$OUT/run_$(pad $i)_stack.log"
  echo "[run $i] $(date +%H:%M:%S) starting"

  teardown

  # --- Isaac side ---
  ROS_DOMAIN_ID=77 KOTEK_WITH_ROS=1 ./run_isaac.sh \
    src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage.py --duration 1400 \
    > "$SLOG" 2>&1 &
  ISAAC_PID=$!
  up=0
  for _ in $(seq 1 36); do
    grep -q '### physics playing' "$SLOG" 2>/dev/null && { up=1; break; }
    sleep 5
  done
  if [ "$up" -ne 1 ]; then
    echo "[run $i] Isaac did not reach physics playing"
    cp "$SLOG" "$OUT/fail_logs/run_$(pad $i)_stage.log"
    record INFRA 0 - - 'isaac_no_physics' infra_isaac
    kill $ISAAC_PID 2>/dev/null
    continue
  fi

  # --- container stack ---
  docker compose -f docker/compose.yaml run --rm -e ROS_DOMAIN_ID=77 kotek bash -lc \
    'source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install --packages-skip scout_nav2_pkg 2>&1 | tail -2 && source install/setup.bash && exec ros2 launch kotek_bringup kotek_wall_mount_demo.launch.py' \
    > "$KLOG" 2>&1 &
  STACK_PID=$!
  CNAME=""
  for _ in $(seq 1 48); do
    grep -q 'piper_manipulator ready' "$KLOG" 2>/dev/null && break
    sleep 5
  done
  CNAME=$(docker ps --format '{{.Names}}' | grep kotek-run | head -1)
  if ! grep -q 'piper_manipulator ready' "$KLOG" 2>/dev/null || [ -z "$CNAME" ]; then
    echo "[run $i] stack not ready"
    cp "$KLOG" "$OUT/fail_logs/run_$(pad $i)_stack.log"
    record INFRA 0 - - 'stack_not_ready' infra_stack
    kill $STACK_PID 2>/dev/null; kill $ISAAC_PID 2>/dev/null
    continue
  fi

  sleep 45  # warm-up, per README 7.2

  # --- task ---
  timeout 900 docker exec "$CNAME" bash -lc \
    'source /opt/ros/jazzy/setup.bash && cd /workspace && source install/setup.bash && ros2 run kotek_wall_mount_task wall_mount_task' \
    > "$TLOG" 2>&1
  rc=$?

  # --- analyze ---
  welds=$(grep -c 'welded sensor' "$SLOG" 2>/dev/null || echo 0)
  mis=$(grep 'welded sensor' "$SLOG" 2>/dev/null | grep -oE 'misalign_deg=[0-9.]+' | cut -d= -f2 | paste -sd'|' -)
  [ -z "$mis" ] && mis=-
  if grep -q 'wall-mount task COMPLETE' "$TLOG"; then result=COMPLETE
  elif grep -q 'wall-mount task ABORTED' "$TLOG"; then result=ABORTED
  elif [ $rc -eq 124 ]; then result=TIMEOUT
  else result=ERROR_RC_$rc; fi
  abort_stage=$(grep -oE '(grasp|place)\[[0-9]\]: FAILED -- [A-Z_]+' "$TLOG" 2>/dev/null | tail -1 | sed 's/.*FAILED -- //')
  [ -z "$abort_stage" ] && abort_stage=-
  err=$(grep -E 'FAILED --' "$TLOG" 2>/dev/null | tail -1 | sed 's/.*FAILED -- //;s/,/,/g' | tr ',' ';' | cut -c1-120)
  [ -z "$err" ] && err=-

  # classification
  if [ "$result" = COMPLETE ] && [ "$welds" -eq 4 ]; then cls=pass
  elif [ "$result" = COMPLETE ]; then
    cls=partial_weld
    # distinguish weld-gate reject (box parked at wall, in_range, never welded)
    # from mid-carry slip (box fell away from the wall)
    missing=$(for s in 1 2 3 4; do grep -q "welded sensor $s " "$SLOG" || echo $s; done)
    for s in $missing; do
      last=$(grep "sensor$s:" "$SLOG" | tail -1)
      if echo "$last" | grep -q 'in_range=True'; then
        cls="${cls}+gate_reject[s$s]"
      else
        cls="${cls}+slip[s$s]"
      fi
    done
  elif grep -q 'Cannot push a new trajectory' "$KLOG" 2>/dev/null || grep -q 'process has died' "$KLOG" 2>/dev/null; then
    cls=move_group_crash
  elif [ "$result" = ABORTED ]; then
    case "$abort_stage" in
      MOVE_ARM_TO_PREGRASP|MOVE_ARM_TO_GRASP|CLOSE_GRIPPER|LIFT_OBJECT) cls=grasp_fail ;;
      *) cls=place_fail ;;
    esac
  else
    cls=task_timeout
  fi

  if [ "$cls" != pass ]; then
    cp "$TLOG" "$OUT/fail_logs/run_$(pad $i)_task.log" 2>/dev/null
    cp "$SLOG" "$OUT/fail_logs/run_$(pad $i)_stage.log" 2>/dev/null
  fi

  record "$result" "$welds" "$mis" "$abort_stage" "$err" "$cls"
  echo "[run $i] $result welds=$welds cls=$cls ($(( $(date +%s) - t0 ))s)"

  kill $ISAAC_PID 2>/dev/null
  kill $STACK_PID 2>/dev/null
  docker rm -f "$CNAME" >/dev/null 2>&1
done

echo "=== batch done: $CSV ==="
column -s, -t < "$CSV"
