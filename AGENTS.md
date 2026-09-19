# Repository Guidelines

## Project Structure & Module Organization

HoloLib is a PROS project for VEX V5 robots with holonomic drivetrain support.
- `src/main.cpp` defines hardware, initialization, autonomous routines, and driver control.
- `src/robot/`, `src/motions/`, `src/localization/`, and `src/util/` implement chassis control, motion routines, localization, and shared utilities. Public headers live under `include/hololib/`.
- `src/liftlib/` and `include/liftlib/` contain mechanism controllers.
- `include/pros/`, `include/Eigen/`, `include/liblvgl/`, and `firmware/` contain bundled dependencies; avoid unrelated edits there.
- `tools/sim_auton.py` generates a browser route viewer using `tools/img/map.png`. `docs/` contains the usage guide, diagrams, and Doxygen configuration. Generated binaries and viewer files belong in ignored `bin/`.

## Build, Test, and Development Commands

Run commands from the repository root unless noted. Install PROS CLI and an `arm-none-eabi` toolchain supporting the configured `gnu++26` standard.
- `make`: build the default incremental `quick` target.
- `make clean`: remove build outputs before rebuilding.
- `pros upload`: upload the built program to a connected V5 brain; robot behavior runs on hardware.
- `python3 tools/sim_auton.py`: generate `bin/auton_viewer.html`; open it in a browser to inspect routes.
- `clang-format -i src/path/file.cpp include/hololib/path/file.hpp`: format changed C++ files.

Documentation builds follow `.github/workflows/doxygen.yml`, which prepares theme assets before running Doxygen from `docs/`.

## Coding Style & Naming Conventions

Follow `.clang-format`: LLVM style, four-space indentation, 120-column limit, and left-aligned pointers. Use PascalCase for types and camelCase for functions; preserve existing module-specific filenames and member prefixes. Use `.hpp` for library headers and `.cpp` for implementations. Document public APIs with Doxygen comments. `.clang-tidy` enables compiler diagnostics, analyzer checks, and C++ Core Guidelines checks.

## Testing Guidelines

No automated test framework, test naming convention, or coverage threshold is configured. Build C++ changes and inspect affected routes with the viewer. Verify control, localization, and mechanism changes on hardware. Record starting pose, gains, expected behavior, and observed results; simulation alone does not validate sensor or motor behavior.

## Commit & Pull Request Guidelines

History uses short descriptive subjects, such as `Updated controls and stuff`; no formal commit convention is established. Prefer focused, actionable subjects such as `Fix lift hold feedforward`. Describe the problem, affected modules, validation, and hardware configuration changes in pull requests. Link relevant issues and include screenshots for viewer or documentation changes.

## Hardware Configuration

Keep global definitions consistent with `include/hololib/config.hpp`. Review motor ports, reversal signs, dimensions, and gains in `src/main.cpp` before hardware runs. Define the vision sensor before enabling AprilTag localization.
