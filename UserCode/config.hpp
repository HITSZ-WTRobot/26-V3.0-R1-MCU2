#pragma once

#include "motor_pos_controller.hpp"

namespace AppConfig
{

namespace Clamp
{
inline constexpr float MaxVelRoll = 100.0f;
inline constexpr float MaxVelYaw  = 10.0f;

inline constexpr float OutManualSpeed  = 100.0f;
inline constexpr float YawManualSpeed  = 50.0f;
inline constexpr float RollManualSpeed = 150.0f;

inline constexpr controllers::MotorPosController::Config kM2006PosConfig{
    .position_pid       = {.Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f},
    .velocity_pid       = {.Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 8000.0f},
    .pos_vel_freq_ratio = 10U,
};

inline constexpr controllers::MotorPosController::Config kM3508PosConfig{
    .position_pid       = {.Kp = 3.0f, .Ki = 0.0f, .Kd = 3.0f, .abs_output_max = 400.0f},
    .velocity_pid       = {.Kp = 450.0f, .Ki = 1.00f, .Kd = 0.0f, .abs_output_max = 8000.0f},
    .pos_vel_freq_ratio = 10U,
};
} // namespace Clamp

} // namespace AppConfig
