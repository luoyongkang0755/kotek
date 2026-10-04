# Wall-mount demo recordings (2026-09-23 ~ 10-04)

E2E recordings of the contact-holddown wall-mount demo. Current final
config (F group, 2026-09-29): axis-aligned sensor boxes + 4 s hand-timed
slow close + grasp_width 0.018 + riser rubber 1.6/1.4 + OMPL place
fallback -- validated 60/60 COMPLETE, 240 grasps 0 kicks, 240 welds all
0.0 deg (10-run + 50-run batches, docs/wall_mount_progress_report.md).

| File | Content | Result |
|---|---|---|
| `wall_mount_final_boxyaw0_4of4.mp4` | **Primary (current final config, F group).** Viewport capture: full run on the axis-aligned-box config. All four boxes grasped first-try (no kick, no re-grasp), pressed, welded | COMPLETE, 4 welds all d=0.0000 m / 0.0 deg (~343 s) |
| `wall_mount_still_boxyaw0.png` | Frame from the primary run (arm mounting a box; axis-aligned riser boxes visible) | poster frame |
| `wall_mount_final_rubber_4of4.mp4` | Previous primary (D group): full run on 4s slow close + riser rubber, radially-rotated boxes era. One run of the 9/10-COMPLETE config | COMPLETE, 4 welds all d=0.0000 m / 0.0 deg (331 s) |
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
viewport capture is immune to both and excludes all UI overlays. The grasp-phase kick --
the known residual issue of the pre-F configs (PhysX point contacts have no torsional
friction; ~57% of closes kicked pre-slow-close, ~10% with riser rubber) -- is eliminated
at the source by the F group's axis-aligned boxes: the jaws close face-parallel from
first contact, and the 50-run confirmation batch recorded 0 kicks in 200 grasps. See
`docs/wall_mount_contact_placement.md` section 0.
