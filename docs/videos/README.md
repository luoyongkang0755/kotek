# Wall-mount demo recordings (2026-09-23/24)

E2E recordings of the contact-holddown wall-mount demo (final config:
4 s hand-timed slow close + grasp_width 0.018 + OMPL place fallback).
Captured with ffmpeg x11grab from the Isaac Sim GUI run.

| File | Content | Result |
|---|---|---|
| `wall_mount_final_rubber_4of4.mp4` | **Primary (current final config).** Viewport-level capture (no UI, no dialogs): full run on the champion config -- 4s slow close + riser rubber. All four boxes grasped first-try, pressed, welded | COMPLETE, 4 welds all d=0.0000 m / 0.0 deg (331 s) |
| `wall_mount_complete_4of4_cropped.mp4` | Earlier complete run, x11grab whole-desktop era, cropped to the sim window (1920x1080) | COMPLETE, 4 welds all d=0.0000 m / 0.0 deg (261 s). First ~2.5 min have a centered ScriptNode warning dialog (self-dismisses); the remaining ~3 min are clean close-ups |
| `wall_mount_complete_4of4.mp4` | Same successful run, earlier take, whole-desktop capture (small window) | COMPLETE, 4 welds 0.0 deg (231 s) |
| `wall_mount_kick_and_recovery_2of4.mp4` | Full-window take: 2 mounts, then grasp[3] close-kick (85/91 deg) and reactive re-grasp attempts | ABORTED at grasp[3] after 3 attempts (2 welds 0.0 deg) |
| `wall_mount_still.png` | Frame from the primary run (arm carrying toward the wall) | poster frame |

Recording pipeline: `/home/trs/e2e_runs/record_wall_mount.sh`. Capture is
viewport-level (`run_wall_stage.py --gui --record-dir`, PNG frames via
`omni.kit.viewport.utility` assembled to mp4) -- the host migrated to a
Wayland GNOME session on 2026-09-25, after which x11grab reads a black
framebuffer (Xwayland never carries composited pixels) and any new
replicator render product shuts Isaac down on first render; the app-native
viewport capture is immune to both and excludes all UI overlays. The grasp-phase kick is
the known residual issue (PhysX point contacts have no torsional friction;
~35% of closes kick the box pre-rubber, ~10% with the final riser-rubber
config) -- see `docs/wall_mount_contact_placement.md` section 0.
