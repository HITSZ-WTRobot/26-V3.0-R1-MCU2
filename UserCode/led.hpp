#pragma once

#include <cstdint>

namespace Led
{

// 初始化 LED/继电器控制模块（预留初始化接口）。
void app_led_init();
// 选择一路继电器（0x01..0x04），并通过 Modbus-RTU 关闭其他三路。
void relay_select(uint8_t relay_value);

} // namespace Led
