#include "controller.hpp"

#include "flags.hpp"
#include "stm32f4xx_hal_uart.h"
#include "watchdog.hpp"

#include <cstdlib>
#include <cstdint>
#include <string.h>

namespace Controller
{

using ProjectControllerConfig = AppConfig::Controller;
using ProjectClampConfig      = AppConfig::Clamp;

static uint8_t readIndex  = 0;
static uint8_t writeIndex = 0;

uint8_t        buffer[ProjectControllerConfig::RawDataSize];
static uint8_t RX_RING_BUFFER[ProjectControllerConfig::RingBufferSize];
uint32_t       decode_count            = 0;
uint32_t       decode_error_count      = 0;
uint32_t       decode_success_count    = 0;
bool           is_controller_connected = true;

static service::Watchdog controller_watchdog;

float LX_T = 0.0f;
float LY_T = 0.0f;
float RX_T = 0.0f;

int16_t LX = 0;
int16_t LY = 0;
int16_t RX = 0;
int16_t RY = 0;

uint32_t button     = 0;
uint8_t  DIP_switch = 0;
uint8_t  crc        = 0;

osThreadId_t         controllerHandle;
const osThreadAttr_t controller_attributes = {
    .name       = "controller",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityHigh,
};

static uint8_t CRC8(const uint8_t* data, uint8_t len)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        value ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
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

static void Msg_AddReadIndex(uint8_t length)
{
    readIndex = (uint8_t)((readIndex + length) % ProjectControllerConfig::RingBufferSize);
}

static uint8_t Msg_Read(uint8_t offset)
{
    const uint8_t index =
            (uint8_t)((readIndex + offset) % ProjectControllerConfig::RingBufferSize);
    return RX_RING_BUFFER[index];
}

static uint8_t Msg_GetLength()
{
    return (uint8_t)((writeIndex + ProjectControllerConfig::RingBufferSize - readIndex) %
                     ProjectControllerConfig::RingBufferSize);
}

static uint8_t Msg_GetRemain()
{
    return (uint8_t)(ProjectControllerConfig::RingBufferSize - Msg_GetLength());
}

static uint8_t Msg_Write(uint8_t* data, uint8_t length)
{
    if (Msg_GetRemain() < length)
    {
        return 0;
    }

    if (writeIndex + length < ProjectControllerConfig::RingBufferSize)
    {
        memcpy(RX_RING_BUFFER + writeIndex, data, length);
        writeIndex = (uint8_t)(writeIndex + length);
    }
    else
    {
        const uint8_t firstPart = (uint8_t)(ProjectControllerConfig::RingBufferSize - writeIndex);
        memcpy(RX_RING_BUFFER + writeIndex, data, firstPart);
        memcpy(RX_RING_BUFFER, data + firstPart, length - firstPart);
        writeIndex = (uint8_t)(length - firstPart);
    }

    writeIndex = (uint8_t)(writeIndex % ProjectControllerConfig::RingBufferSize);
    return length;
}

static bool Msg_SyncToHeader()
{
    while (Msg_GetLength() >= ProjectControllerConfig::RawDataSize)
    {
        if (Msg_Read(0) == ProjectControllerConfig::FrameHeader1 &&
            Msg_Read(1) == ProjectControllerConfig::FrameHeader2)
        {
            return true;
        }
        Msg_AddReadIndex(1);
    }

    return false;
}

static void Button_Init()
{
    button     = 0;
    DIP_switch = 0;
}

extern "C" void controller_task(void* argument)
{
    (void)argument;

    static uint16_t prev_buttons = 0;
    while (1)
    {
        while (Msg_SyncToHeader())
        {
            decode_count++;

            uint8_t receive_data[ProjectControllerConfig::RawDataSize - 3U];
            for (size_t i = 0; i < ProjectControllerConfig::RawDataSize - 3U; i++)
            {
                receive_data[i] = Msg_Read((uint8_t)(i + 2U));
            }

            const uint8_t calculate_crc =
                    CRC8(receive_data, (uint8_t)(ProjectControllerConfig::RawDataSize - 3U));
            const uint8_t received_crc = Msg_Read(13);

            if (calculate_crc == received_crc)
            {
                DIP_switch = Msg_Read(10);

                const uint16_t curr_buttons =
                        (uint16_t)((static_cast<uint16_t>(Msg_Read(11)) << 8) | Msg_Read(12));
                const uint16_t falling_buttons = (uint16_t)(prev_buttons & ~curr_buttons);

                button = static_cast<uint32_t>(curr_buttons) |
                         (static_cast<uint32_t>(DIP_switch) << 16);
                const uint32_t event_flags =
                        static_cast<uint32_t>(falling_buttons) |
                        (static_cast<uint32_t>(DIP_switch) << 16);
                osEventFlagsSet(flags_id, event_flags);
                prev_buttons = curr_buttons;

                if (reset_status == success)
                {
                    if (button & 0x00000010U)
                    {
                        clamp_vel_out = -ProjectClampConfig::OutManualSpeed;
                    }
                    else if (button & 0x00000020U)
                    {
                        clamp_vel_out = ProjectClampConfig::OutManualSpeed;
                    }
                    else
                    {
                        clamp_vel_out = 0.0f;
                    }
                }

                if (button & 0x00000001U)
                {
                    clamp_vel_yaw = ProjectClampConfig::YawManualSpeed;
                }
                else if (button & 0x00000100U)
                {
                    clamp_vel_yaw = -ProjectClampConfig::YawManualSpeed;
                }
                else
                {
                    clamp_vel_yaw = 0.0f;
                }

                if (button & 0x00000002U)
                {
                    clamp_vel_roll = -ProjectClampConfig::RollManualSpeed;
                }
                else if (button & 0x00000200U)
                {
                    clamp_vel_roll = ProjectClampConfig::RollManualSpeed;
                }
                else
                {
                    clamp_vel_roll = 0.0f;
                }

                decode_success_count++;
                controller_watchdog.feed(500);
                Msg_AddReadIndex(ProjectControllerConfig::RawDataSize);
            }
            else
            {
                Msg_AddReadIndex(2);
                decode_error_count++;
            }
        }

        osDelay(1);
    }
}

void app_controller_receive_init(void)
{
    Button_Init();
    osThreadNew(controller_task, NULL, &controller_attributes);
    HAL_UART_Receive_DMA(&huart1, buffer, ProjectControllerConfig::RawDataSize);
}

void ControllerReceive_OnRxCplt()
{
    Msg_Write(buffer, ProjectControllerConfig::RawDataSize);
}

void update_1kHz()
{
    if (!controller_watchdog.isFed())
    {
        is_controller_connected = false;
    }
    else
    {
        is_controller_connected = true;
    }
}

} // namespace Controller
