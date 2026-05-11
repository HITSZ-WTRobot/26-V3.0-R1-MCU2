# Repository Guidelines

## Project Structure & Module Organization
`UserCode/` contains application-owned firmware logic and is the default place for new features. Keep paired source/header files together, for example `controller.cpp` and `controller.hpp`. `Core/`, `Drivers/`, `Middlewares/`, `startup_stm32f407xx.s`, `STM32F407XX_FLASH.ld`, and `26-V3.0-R1-MCU2.ioc` come from STM32CubeMX or board setup; treat them as generated integration layers and keep manual edits minimal. Reusable dependencies live under the submodules `Modules/BasicComponents` and `Modules/MotorDrivers`; package links are declared in `wtrproject.toml` and materialized through `cmake/wtr_modules.cmake`.

## Build, Test, and Development Commands
Initialize dependencies before the first build:
`git submodule update --init --recursive`

Configure and build with the shipped presets:
`cmake --preset Debug`
`cmake --build --preset Debug`
`cmake --preset Release`
`cmake --build --preset Release`

If CMake reports a missing managed package, run `cpkg sync` and reconfigure. Build output and `compile_commands.json` are generated under `build/<preset>/`.

## Coding Style & Naming Conventions
The root `CMakeLists.txt` builds C11 and C++17. Follow `.clang-format`: 4-space indentation, no tabs, Allman braces, and a 100-column limit. Use lowercase file names with underscores where needed (`device.hpp`, `motor_vel_controller.cpp`), `PascalCase` for types, and `snake_case` for functions and variables. Prefer keeping ISR callbacks, RTOS entry points, and hardware-facing code explicit and local rather than hidden behind macros.

## Testing Guidelines
There is no repository-wide `ctest` target yet. Treat a clean `Debug` build as the minimum validation for every change, and rebuild `Release` for timing- or size-sensitive work. When adding tests, place them next to the affected module or add a dedicated CMake target, and name files after the unit under test, such as `controller_test.cpp`.

## Commit & Pull Request Guidelines
Current history is short but already uses a typed prefix (`feat:init`). Continue with concise messages like `feat(controller): add clamp watchdog` or `fix(device): guard null motor handles`. Pull requests should summarize the firmware behavior change, list touched generated files separately from hand-written code, and state how the change was validated on build or hardware.

## Maintenance Notes
Keep `AGENTS.md` in sync with the repository workflow. When project requirements, contributor expectations, build steps, or module ownership change, update this file in the same change set so the guide stays accurate.
