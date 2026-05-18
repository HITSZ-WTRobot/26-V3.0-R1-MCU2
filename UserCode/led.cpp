#include "led.hpp"

#include "cmsis_os2.h"
#include "gpio.h"
#include "usart.h"

#include <cstddef>
#include <cstdint>

namespace Led
{

namespace
{
inline constexpr size_t modbus_frame_size = 8U;
inline constexpr uint8_t modbus_address  = 0x01U;
inline constexpr uint8_t modbus_write_coil = 0x05U;
inline constexpr uint16_t coil_base_address = 0x0000U;
inline constexpr uint32_t relay_queue_depth = 4U;

osMessageQueueId_t relay_queue = nullptr;
osThreadId_t relay_task_id = nullptr;

const osThreadAttr_t relay_task_attributes = {
    .name       = "led_relay",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

void relay_power_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {};
    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

// 计算 Modbus-RTU CRC16（多项式 0xA001，初值 0xFFFF）。
uint16_t crc16_modbus(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}
//如果以机械臂伸出的方向为北 0对应东方向灯带 1北方 2西方 3南方
// 组包并通过 UART2 发送 Modbus-RTU 写单线圈（0x05）指令。
// 十六进制帧（地址=0x01，功能码=0x05，CRC 低字节在前）：
// 线圈 0 打开: 01 05 00 00 FF 00 8C 3A
// 线圈 0 关闭: 01 05 00 00 00 00 CD CA
// 线圈 1 打开: 01 05 00 01 FF 00 DD FA
// 线圈 1 关闭: 01 05 00 01 00 00 9C 0A
// 线圈 2 打开: 01 05 00 02 FF 00 2D FA
// 线圈 2 关闭: 01 05 00 02 00 00 6C 0A
// 线圈 3 打开: 01 05 00 03 FF 00 7C 3A
// 线圈 3 关闭: 01 05 00 03 00 00 3D CA
void send_write_coil(uint16_t coil_address, bool on)
{
    uint8_t frame[modbus_frame_size] = {0};
    frame[0] = modbus_address;
    frame[1] = modbus_write_coil;
    frame[2] = static_cast<uint8_t>((coil_address >> 8U) & 0xFFU);
    frame[3] = static_cast<uint8_t>(coil_address & 0xFFU);
    frame[4] = on ? 0xFFU : 0x00U;
    frame[5] = 0x00U;

    const uint16_t crc = crc16_modbus(frame, 6U);
    frame[6] = static_cast<uint8_t>(crc & 0xFFU);
    frame[7] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);

    HAL_UART_Transmit(&huart6, frame, static_cast<uint16_t>(modbus_frame_size), 50U);
}

void relay_apply(uint8_t relay_value)
{
    if (relay_value < 0x01U || relay_value > 0x04U)
    {
        return;
    }

    const uint8_t relay_to_coil[4] = {2U, 1U, 0U, 3U};
    const uint8_t target_index = relay_to_coil[static_cast<uint8_t>(relay_value - 0x01U)];
    for (uint8_t index = 0U; index < 4U; index++)
    {
        const uint16_t coil_address = static_cast<uint16_t>(coil_base_address + index);
        const bool turn_on = (index == target_index);
        send_write_coil(coil_address, turn_on);
        osDelay(100U); // 每帧间隔 100 ms，避免总线过载
    }
}

void relay_task(void* argument)
{
    (void)argument;

    uint8_t relay_value = 0U;
    while (1)
    {
        if (osMessageQueueGet(relay_queue, &relay_value, nullptr, osWaitForever) == osOK)
        {
            while (osMessageQueueGet(relay_queue, &relay_value, nullptr, 0U) == osOK)
            {
            }

            relay_apply(relay_value);
        }
    }
}
} // namespace

void app_led_init()
{
    relay_power_gpio_init();

    if (relay_queue == nullptr)
    {
        relay_queue = osMessageQueueNew(relay_queue_depth, sizeof(uint8_t), nullptr);
    }

    if (relay_queue != nullptr && relay_task_id == nullptr)
    {
        relay_task_id = osThreadNew(relay_task, nullptr, &relay_task_attributes);
    }

    for (uint8_t index = 0U; index < 4U; index++)
    {
        const uint16_t coil_address = static_cast<uint16_t>(coil_base_address + index);
        send_write_coil(coil_address, true);
        osDelay(30U); // 每帧间隔 30ms，避免总线过载
    }
}

// 选择指定继电器（0x01..0x04）打开，并关闭其余三路。
// 例如 relay_value = 0x01 时，依次发送四帧：
// 01 05 00 00 FF 00 8C 3A
// 01 05 00 01 00 00 9C 0A
// 01 05 00 02 00 00 6C 0A
// 01 05 00 03 00 00 3D CA
void relay_select(uint8_t relay_value)
{
    if (relay_queue == nullptr)
    {
        return;
    }

    if (osMessageQueuePut(relay_queue, &relay_value, 0U, 0U) != osOK)
    {
        uint8_t dropped = 0U;
        (void)osMessageQueueGet(relay_queue, &dropped, nullptr, 0U);
        (void)osMessageQueuePut(relay_queue, &relay_value, 0U, 0U);
    }
}

} // namespace Led
