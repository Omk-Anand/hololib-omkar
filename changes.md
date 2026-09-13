# Merge notes: Ashmit's config onto Omkar's codebase

Baseline: `9a36b8d inital commit` (Omkar). Everything below is the delta.

Context worth knowing: the two repos were **not** different codebases. Every file
under `src/` and `include/hololib/`, plus `Makefile`, `common.mk`, `project.pros`,
`README.md`, `docs/`, `tools/`, `.github/` and the clang configs, was already
byte-identical between them. The only real differences were `src/main.cpp` and the
absence of liftlib. So this merge touches `main.cpp` and adds liftlib — nothing in
the library itself changed.

Verified with a clean `make all`: compiles and links, no errors, no warnings.
Runtime behavior is **not** verified — none of this has been on the robot.

---

## 1. Ports

| Motor | Was | Now | Why |
|---|---|---|---|
| `backRPort` | 19 | **17** | As requested. |
| port 9 | `clawLift` (raw motor) | **`clawRotationLift`** (PID subsystem, reversed) | Adopting Ashmit's layout. |
| port 10 | `clawRotation` (raw motor) | **`clawGripper`** (raw rollers) | Adopting Ashmit's layout. |

**Wiring confirmed on the robot (Ashmit).** Port 9 is the claw's vertical
rotation (the pivot) and port 10 is the flex wheel. Neither touches the cascade
lift, which is ports 6/-7 alone. Omkar's `clawLift` on port 9 was a mislabel --
no separate claw-lift mechanism exists, so nothing needs to replace it.

## 2. PIDs

- **Drivetrain (`xSched`/`ySched`/`thetaSched`) — unchanged.** They were already
  character-for-character identical in both files.
- **`clawRotationPID` — unchanged values** (`kP=15.0, kI=0.0001, kD=16.0,
  threshold=2.0`), also already identical. What changed is that it now actually
  runs (see §4).
- **`liftLift` — new.** Omkar's file had no closed-loop controller for the
  elevator at all; R1/R2 drove it open loop and `my_lift` (ModularLift) is never
  commanded. Added as a `liftlib::Subsystem` on ports 6/-7 with a **position-based
  gain schedule**, targets in inches:
  - 10" → `kP=10.0, kI=0, kD=0, threshold=1.0"`
  - 19" → `kP=1.0,  kI=0, kD=0, threshold=1.0"`
  - `inches = motor_degrees * gear_ratio * (pi/180) * spool_radius`, with
    `LIFT_GEAR_RATIO = 1.0` (direct drive) and `LIFT_SPOOL_RADIUS_IN = 0.4`.
    Both measured; remeasure if the spool or gearing changes.
  - Gains are **starting points, not tuned.** kD is 0 on both — expect overshoot.
- **Feedforwards — new.** `clawRotationLift` gets `Feedforward::cosine(kG=0,
  horizontal=0)` (it pivots, so hold torque varies with angle); `liftLift` gets
  `Feedforward::constant(kG=18.5)` (a cascade's load doesn't change with height).
  `kG=0` on the claw means **no gravity compensation yet** — raise it until the
  claw holds level without sagging.

## 3. Controls (`opcontrol`)

| Binding | Was | Now |
|---|---|---|
| R1 / R2 | lift up/down, open loop | unchanged |
| DOWN / RIGHT | intake in/out | **removed** — folded into B/Y, both buttons now free |
| B / Y | `clawLift` up/down | **intake + gripper together**, one button per direction |
| L1 / L2 | `clawRotation` raw voltage | **`clawRotationLift.setOutput(±60)`**, releases into `holdActively()` |

Two behavior changes beyond the bindings:

- **`odom.setKalmanFilterEnabled(false)` → `true`.** With the EKF off, heading
  integrates from wheel encoders alone and drifts fast on an X-drive with no
  tracking wheels.
- **`driveControl(..., fieldCentric)` `false` → `true`.** "Forward" now means the
  direction the bot faced when opcontrol started. This depends on the EKF change
  above — the flag is useless without it.

Also removed a stray `pros::lcd::print(6,"test")` that fought the new line-6 readout.

## 4. Compile fixes

Omkar's `main.cpp` **did not build.** Three separate problems:

1. **Missing include.** It used `liftlib::PID`, `liftlib::Subsystem` and
   `liftlib::MotorConfig` with no `#include "liftlib/liftlib.hpp"`. Added, behind
   `#define LIFTLIB_NO_GLOBAL_NAMES` so liftlib's `PID` doesn't collide with the
   existing `hololib::PID`.
2. **No liftlib in the tree.** `liftLib/` at the repo root was a **broken git
   submodule** — a gitlink (mode 160000, commit `1a36a98`) with no `.gitmodules`
   file, so it cloned as an empty directory with no URL to resolve it from.
   Removed the dead gitlink and vendored the real sources (see §5).
3. **Dead objects.** `clawRotationPID` and `clawRotationLift` were declared as
   **locals inside `initialize()`** — constructed and immediately destroyed on
   return, controlling nothing. Moved to file scope, where they outlive the
   function and can be reached from `autonomous()` and `opcontrol()`.

## 4b. visionSensor left undefined (deliberately)

`config.hpp` declares `extern pros::AIVision visionSensor`, and neither repo
defined it. The robot has no AI Vision sensor right now, so it stays undefined --
but the definition is written out and commented in `main.cpp` rather than absent,
so the next person knows where it goes.

This links today only because nothing references the symbol. Constructing an
`ApriltagLocalization` turns it into an undefined-reference link error. Uncomment
`visionPort` and the definition, with the real port, before adding AprilTag work.

## 4c. Removed my_lift (dead ModularLift)

`main.cpp` had **three** objects bound to lift motors 6/-7: `liftMotors` (the raw
MotorGroup that R1/R2 actually jog), the new `liftLift` PID subsystem, and
`my_lift`, a `ModularLift` with a full `LiftConfig` and LQR gain matrix that was
constructed and then only ever `.cancel()`ed in `disabled()`. It commanded the
lift nowhere.

Removed `my_lift`, `my_lift_config`, `lift_motor_configs`, the `.cancel()` call,
and the now-unused `#include "hololib/util/modular_lift.hpp"`. Two reasons: it
read as the lift controller when it wasn't, and three owners of one pair of motors
is a real hazard the moment someone commands two of them.

**Omkar:** this was your code and your LQR tuning, deleted on Ashmit's call, not
because it was wrong. `include/hololib/util/modular_lift.hpp` and
`src/motions/controllers/modular_lift.cpp` are untouched, so it is a revert of this
block away if you want it back -- but decide which framework owns the lift first.

## 5. Structure

Deliberately minimal — only what was needed to build.

- **Removed** `liftLib/` (the dead, empty submodule gitlink).
- **Added** `include/liftlib/` (9 headers) and `src/liftlib/` (5 sources), vendored
  as real files. This location is not arbitrary: the Makefile sets `SRCDIR=src` and
  `INCDIR=include`, so a root-level `liftLib/` is never compiled or searched. It
  also mirrors how `hololib` is already laid out in this repo.
- **Added** `initialize()` calls to tare both subsystems at boot, and LCD lines 6/7
  showing live claw angle and lift height (for tuning).
- **Untouched:** every library file, `my_lift`/`LiftConfig`/`ModularLift`,
  `autonomous()`, `disabled()`, `simulation()`, the drivetrain gains, the LCD task,
  and all build files.

## Not ported

- `liftlib::Lift liftAssembly` (grouping both stages under one object) — works, but
  adds a concept for no current benefit; nothing calls a group move.
- Ashmit's claw/lift **tuning routines in `autonomous()`** — Omkar's simpler
  `turnToHeading(90)` autonomous was kept as-is.

## Open items

1. ~~Confirm the port 9/10 wiring~~ **Done** -- confirmed correct, see §1.
2. ~~Confirm `visionPort`~~ **N/A** -- no sensor on the robot; commented out (see §4b).
3. ~~Confirm the claw pivot direction~~ **Done** -- confirmed by Ashmit: L1 raises the
   claw as intended, so `.port = -9` is correct as written.
4. ~~Verify `LIFT_SPOOL_RADIUS_IN` and `LIFT_GEAR_RATIO`~~ **Done** -- confirmed by
   Ashmit: spool radius 0.4", no gearing on the lift, so ratio 1.0. The
   motor-degrees-to-inches conversion is correct as written.
5. **Tune `liftLift`** — kD is 0 at both gain points. Gains live in the
   `GainPoint` list in `liftLift`'s declaration in `main.cpp`.
6. **Tune `clawRotationLift`'s `kG`** — currently 0, so no gravity compensation.
7. ~~Nothing calls `liftLift.moveTo()`~~ **Done** — `autonomous()` now does a blocking
   `liftLift.moveTo(10.0f, false, 5000)` so the lift PID can be tuned against LCD
   line 7. The `turnToHeading(90)` is commented out during tuning.
8. ~~`my_lift` is dead code~~ **Done** -- removed, see §4c.
9. ~~`CLAUDE.md` untracked~~ **Done** -- updated for this codebase and added to
   `.gitignore`, so it stays local and out of the shared repo.
