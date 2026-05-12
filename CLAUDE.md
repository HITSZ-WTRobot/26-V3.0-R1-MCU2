# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Platform & Toolchain

STM32F407xx (Cortex-M4F, 168 MHz) bare-metal firmware using CMSIS-RTOS2. Cross-compiled with `arm-none-eabi-gcc` via CMake+Ninja.

Build:
```bash
cmake --preset Debug       # or Release
cmake --build --preset Debug
```

Before first build: `git submodule update --init --recursive`. If CMake reports missing managed packages, run `cpkg sync` and reconfigure.

`compile_commands.json` lands in `build/<preset>/`. The `.clangd` file points there for LSP.

## Project Structure

```
UserCode/            Application firmware — the primary workspace for new code
  app.cpp            Entry point: RTOS init thread, 1kHz timer ISR, UART RX callback
  device.cpp/.hpp    CAN bus setup + 7 DJI motor instances across hcan1/hcan2
  controller.cpp/.hpp  UART remote-control protocol (header `0xAA 0xBB`, 14-byte frames, CRC8)
  clamp.cpp/.hpp     Clamp mechanism: dual-mode pos/vel PID per axis, reset sequence FSM
  config.hpp         Compile-time constants (manual speed, limits)
  arena.cpp          Global operator new/delete via static linear arena (64 KB, no free)
  flags.cpp/.hpp     CMSIS-RTOS2 event flags for cross-task signaling

Core/ Drivers/ Middlewares/  STM32CubeMX-generated HAL layer — minimize manual edits
startup_stm32f407xx.s        Startup code (kept in repo root, not gitignored)
STM32F407XX_FLASH.ld         Linker script
26-V3.0-R1-MCU2.ioc          CubeMX project file

Modules/BasicComponents/     Git submodule: utils, PID controllers, CAN driver, watchdog
Modules/MotorDrivers/        Git submodule: DJI/DM/VESC motor drivers, controller library
```

Dependencies declared in `wtrproject.toml`, materialized through `cmake/wtr_modules.cmake`.

## Application Architecture

**Timing**: A hardware timer fires at 1 kHz (`TIM_Callback_1kHz`). The ISR calls in order:
1. `Watchdog::EatAll()` — service-level liveness check
2. `Controller::update_1kHz()` — checks if controller data is fresh (watchdog-based)
3. `Device::update_1kHz()` — sends CAN Iq commands (3 groups across 2 buses)
4. `APP_Clamp_Update_1kHz()` — runs PID control loops for all 4 clamp axes

**Threads** (CMSIS-RTOS2):
- `controller_task` (priority High): decodes UART ring buffer frames, sets event flags and velocity setpoints from button state
- `Clamp_Control` (priority Normal1): 100 Hz loop handling reset sequence FSM
- `Clamp_softTIM` (timer callback, 20 ms period): reads event flags to toggle control_reset and catch_angle

**CAN layout**: hcan1 carries the manipulator motors (IDs 1-3), hcan2 carries the clamp motors (IDs 1-4, with CAN filter offset 14). DJI motor types are M3508 (C620 ESC) and M2006 (C610 ESC).

**Memory**: No heap. `operator new` is overridden to use a 64 KB static linear arena. No `free`/`delete` — memory is never reclaimed. No exceptions, no RTTI, no thread-safe statics (`-fno-rtti -fno-exceptions -fno-threadsafe-statics`).

**Controller protocol**: 14-byte UART frames with `0xAA 0xBB` header and CRC8. DMA receives into a circular ring buffer (64 bytes). The decode thread syncs to header, validates CRC, and maps button/dip-switch bits to clamp velocity setpoints.

**Clamp control**: Each of the 4 axes (out, roll, yaw, catch) has both a position PID cascade and a velocity PID controller. The out and roll axes switch between pos/vel mode at runtime. The reset sequence drives the out axis against a mechanical stop to zero the angle.

## Coding Conventions

C11 + C++17. `.clang-format` enforces: 4-space indent, no tabs, Allman braces, 100-column limit, pointer alignment left (`int* p`). Filenames: lowercase with underscores. Types: `PascalCase`. Functions/variables: `snake_case`. No iostream, no STL containers (bare-metal constraints apply).

FreeRTOS / CMSIS-OS calls are explicit — no wrapper macros. ISR callbacks should stay minimal; defer heavy work to threads.

## Testing

No `ctest` target exists. A clean Debug build is the minimum validation. For timing/size-sensitive changes, also build Release.

## Commit Style

Typed prefix: `feat(scope): description` or `fix(scope): description`. Separate generated files from hand-written code in PR descriptions.
