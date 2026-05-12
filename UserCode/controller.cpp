#include "controller.hpp"
#include "arm.hpp"

#include "clamp.hpp"
#include "config.hpp"
#include "flags.hpp"
#include "usart.h"
#include "watchdog.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#define KEY1 0x00000001U
#define KEY2 0x00000002U
#define KEY3 0x00000004U
#define KEY4 0x00000008U
#define KEY5 0x00000010U
#define KEY6 0x00000020U
#define KEY7 0x00000040U
#define KEY8 0x00000080U
#define KEY9 0x00000100U
#define KEY10 0x00000200U
#define KEY11 0x00000400U
#define KEY12 0x00000800U

namespace Controller
{

namespace
{
namespace clamp_config = AppConfig::Clamp;

inline constexpr uint8_t raw_data_size      = 13U;
inline constexpr uint8_t ring_buffer_size   = 64U;
inline constexpr uint8_t frame_header_1     = 0xAAU;
inline constexpr uint8_t frame_header_2     = 0xBBU;
inline constexpr uint8_t dip_switch_index   = 9U;
inline constexpr uint8_t button_high_index  = 10U;
inline constexpr uint8_t button_low_index   = 11U;
inline constexpr uint8_t crc_index          = 12U;
inline constexpr uint8_t payload_offset     = 2U;
inline constexpr uint8_t payload_size       = raw_data_size - 3U;//参与crc校验的实际字节数
inline constexpr uint32_t watchdog_feed_ttl = 500U;

uint8_t  rx_dma_buffer[raw_data_size];  
uint8_t  rx_ring_buffer[ring_buffer_size];
uint8_t  read_index              = 0U;
uint8_t  write_index             = 0U;
uint32_t decode_count            = 0U;
uint32_t decode_error_count      = 0U;
uint32_t decode_success_count    = 0U;
bool     is_controller_connected = true; 
uint32_t button_state            = 0U;
uint8_t  dip_switch              = 0U;

const osThreadAttr_t controller_attributes = {
    .name       = "controller",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityHigh,
};

service::Watchdog controller_watchdog;

uint8_t crc8(const uint8_t* data, uint8_t len)
{
    uint8_t value = 0U;

    for (uint8_t i = 0U; i < len; i++)
    {
        value ^= data[i];
        for (uint8_t j = 0U; j < 8U; j++)
        {
            if (value & 0x80U)
            {
                value = (uint8_t)((value << 1U) ^ 0x07U);
            }
            else
            {
                value <<= 1U;
            }
        }
    }

    return value;
}

void msg_add_read_index(uint8_t length)
{
    read_index = (uint8_t)((read_index + length) % ring_buffer_size);
}

uint8_t msg_read(uint8_t offset)
{
    const uint8_t index = (uint8_t)((read_index + offset) % ring_buffer_size);
    return rx_ring_buffer[index];
}

uint8_t msg_get_length()
{
    return (uint8_t)((write_index + ring_buffer_size - read_index) % ring_buffer_size);
}

uint8_t msg_get_remain()
{
    return (uint8_t)(ring_buffer_size - msg_get_length());
}

uint8_t msg_write(const uint8_t* data, uint8_t length)
{
    if (msg_get_remain() < length)
    {
        return 0U;
    }

    if (write_index + length < ring_buffer_size)
    {
        std::memcpy(rx_ring_buffer + write_index, data, length);
        write_index = (uint8_t)(write_index + length);
    }
    else
    {
        const uint8_t first_part = (uint8_t)(ring_buffer_size - write_index);
        std::memcpy(rx_ring_buffer + write_index, data, first_part);
        std::memcpy(rx_ring_buffer, data + first_part, length - first_part);
        write_index = (uint8_t)(length - first_part);
    }

    write_index = (uint8_t)(write_index % ring_buffer_size);
    return length;
}

bool msg_sync_to_header()
{
    while (msg_get_length() >= raw_data_size)
    {
        if (msg_read(0U) == frame_header_1 && msg_read(1U) == frame_header_2)
        {
            return true;
        }
        msg_add_read_index(1U);
    }

    return false;
}

void button_init()
{
    button_state = 0U;
    dip_switch   = 0U;
}

static void HandleArmControl(uint16_t falling_buttons)
{
    if ((falling_buttons & KEY11) != 0U)
    {
        // Auto_height is defined by DIP switch bits 5 and 6.
        // 00 -> 0, 01 -> 1, 10 -> 2.
        const uint8_t auto_height_bits = (dip_switch >> 5U) & 0x03U;
        ArmAutoCatchLevel auto_height_level = ARM_AUTO_CATCH_LOW;

        if (auto_height_bits == 1U)
        {
            auto_height_level = ARM_AUTO_CATCH_MID;
        }
        else if (auto_height_bits == 2U)
        {
            auto_height_level = ARM_AUTO_CATCH_HIGH;
        }

        Arm_AutoCatchStart(auto_height_level);
    }
    if ((falling_buttons & KEY4) != 0U)
    {
        Arm_Rotate_Out(true);
    }
    if ((falling_buttons & KEY3) != 0U)
    {
        Arm_Rotate_Back(true);
    }
}

static void HandleClampControl(uint32_t btn_state)
{
    if (reset_status == ResetProcess::Success)
    {
        if (btn_state & KEY5)
        {
            clamp_vel_out = -clamp_config::OutManualSpeed;
        }
        else if (btn_state & KEY6)
        {
            clamp_vel_out = clamp_config::OutManualSpeed;
        }
        else
        {
            clamp_vel_out = 0.0f;
        }
    }

    if (btn_state & KEY1)
    {
        clamp_vel_yaw = clamp_config::YawManualSpeed;
    }
    else if (btn_state & KEY9)
    {
        clamp_vel_yaw = -clamp_config::YawManualSpeed;
    }
    else
    {
        clamp_vel_yaw = 0.0f;
    }

    if (btn_state & KEY2)
    {
        clamp_vel_roll = -clamp_config::RollManualSpeed;
    }
    else if (btn_state & KEY10)
    {
        clamp_vel_roll = clamp_config::RollManualSpeed;
    }
    else
    {
        clamp_vel_roll = 0.0f;
    }
}


} // namespace

extern "C" void controller_task(void* argument)
{
    (void)argument;

    static uint16_t previous_buttons = 0U;

    while (1)
    {
        while (msg_sync_to_header())
        {
            decode_count++;

            uint8_t received_payload[payload_size];
            for (size_t i = 0; i < payload_size; i++)
            {
                received_payload[i] = msg_read((uint8_t)(i + payload_offset));
            }

            const uint8_t calculated_crc = crc8(received_payload, payload_size);
            const uint8_t received_crc   = msg_read(crc_index);

            if (calculated_crc == received_crc)
            {
                dip_switch = msg_read(dip_switch_index);

                const uint16_t current_buttons =
                        (uint16_t)((static_cast<uint16_t>(msg_read(button_high_index)) << 8) |
                                   msg_read(button_low_index));
                const uint16_t falling_buttons =
                        (uint16_t)(previous_buttons & ~current_buttons);

                button_state = static_cast<uint32_t>(current_buttons) |
                               (static_cast<uint32_t>(dip_switch) << 16);
                const uint32_t event_flags =
                        static_cast<uint32_t>(falling_buttons) |
                        (static_cast<uint32_t>(dip_switch) << 16);
                osEventFlagsSet(flags_id, event_flags);
                previous_buttons = current_buttons;
                HandleArmControl(falling_buttons);
                HandleClampControl(button_state);

                decode_success_count++;
                controller_watchdog.feed(watchdog_feed_ttl);
                msg_add_read_index(raw_data_size);
            }
            else
            {
                msg_add_read_index(payload_offset);
                decode_error_count++;
            }
        }

        osDelay(1);
    }
}

void app_controller_receive_init(void)
{
    button_init();
    osThreadNew(controller_task, nullptr, &controller_attributes);
    HAL_UART_Receive_DMA(&huart1, rx_dma_buffer, raw_data_size);
}

void ControllerReceive_OnRxCplt()
{
    msg_write(rx_dma_buffer, raw_data_size);
}

void update_1kHz()
{
    is_controller_connected = controller_watchdog.isFed();
}

} // namespace Controller
