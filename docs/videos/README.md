# Wall-mount demo recordings (2026-09-23/24)

E2E recordings of the contact-holddown wall-mount demo (final config:
4 s hand-timed slow close + grasp_width 0.018 + OMPL place fallback).
Captured with ffmpeg x11grab from the Isaac Sim GUI run.

| File | Content | Result |
|---|---|---|
| `wall_mount_complete_4of4.mp4` | Full run, all four boxes mounted | COMPLETE, 4 welds all d=0.0000 m / 0.0 deg (231 s) |
| `wall_mount_kick_and_recovery_2of4.mp4` | Full-window recording: 2 mounts, then grasp[3] close-kick (85 deg/91 deg) and reactive re-grasp attempts | ABORTED at grasp[3] after 3 attempts (2 welds 0.0 deg) |
| `wall_mount_still.png` | Frame from the kick-and-recovery run | poster frame |

Recording pipeline: `/home/trs/e2e_runs/record_wall_mount.sh` (single-Isaac
law, fresh container stack per take, 45 s warm-up, whole-desktop x11grab
3840x2160 -> 1920x1080 H.264). The grasp-phase kick is the known residual
issue (PhysX point contacts have no torsional friction; ~35% of closes kick
the box) -- see `docs/wall_mount_contact_placement.md` section 0.
