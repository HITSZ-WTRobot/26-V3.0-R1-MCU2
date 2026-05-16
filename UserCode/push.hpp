#pragma once

namespace Push
{
extern float target_vel;

inline constexpr float zero_velocity_epsilon = 1e-1f; // 判断速度为0阈值

void app_push_init();
void app_push_update_1kHz();
} // namespace Push
