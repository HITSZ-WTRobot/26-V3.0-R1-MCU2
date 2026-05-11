#pragma once

#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"

#include <cstdint>

namespace AppConfig
{

namespace Controller
{
inline constexpr uint8_t RawDataSize    = 14U;
inline constexpr uint8_t RingBufferSize = 64U;
inline constexpr uint8_t FrameHeader1   = 0xAAU;
inline constexpr uint8_t FrameHeader2   = 0xBBU;
} // namespace Controller

namespace Clamp
{
inline constexpr float MaxVelRoll        = 100.0f;
inline constexpr float MaxVelYaw         = 10.0f;
inline constexpr float AprilTagToMachineX = 0.0f;
inline constexpr float AprilTagToMachineY = 0.0f;

inline constexpr float OutManualSpeed  = 100.0f;
inline constexpr float YawManualSpeed  = 50.0f;
inline constexpr float RollManualSpeed = 150.0f;

inline constexpr controllers::MotorVelController::Config OutVelControllerCfg{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr controllers::MotorVelController::Config YawVelControllerCfg{
    .pid = { .Kp = 45.0f, .Ki = 0.15f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr controllers::MotorVelController::Config RollVelControllerCfg{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr controllers::MotorVelController::Config CatchVelControllerCfg{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 4500.0f },
};

inline constexpr controllers::MotorPosController::Config OutPosControllerCfg{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr controllers::MotorPosController::Config RollPosControllerCfg{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr controllers::MotorPosController::Config YawPosControllerCfg{
    .position_pid       = { .Kp = 80.0f, .Ki = 1.0f, .Kd = 0.0f, .abs_output_max = 2000.0f },
    .velocity_pid       = { .Kp = 12.0f, .Ki = 0.20f, .Kd = 5.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr controllers::MotorPosController::Config CatchPosControllerCfg{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 4500.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr uint32_t SoftTimerPeriodMs = 20U;
inline constexpr uint32_t ControlTaskDelayMs = 100U;
inline constexpr float    ResetSeekVelocity  = 100.0f;
inline constexpr float    ResetLockThreshold = 1500.0f;
inline constexpr uint32_t ResetHoldMs        = 500U;
inline constexpr float    CatchClosedAngle   = 150.0f;
} // namespace Clamp

} // namespace AppConfig
