/* Application initialization */

/**
 * @file    app.h
 * @author  rediduck
 * @date    2026-05-1
 */

#include "cmsis_os2.h"

#include "arm.hpp"
#include "clamp.hpp"
#include "controller.hpp"
#include "device.hpp"
#include "flags.hpp"
#include "main.h"
#include "tim.h"
#include "watchdog.hpp"

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    (void)htim;

    service::Watchdog::EatAll();
    Controller::update_1kHz();
    Device::update_1kHz();
    APP_Clamp_Update_1kHz();
    APP_Arm_Update_1kHz();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Controller::ControllerReceive_OnRxCplt();
    }
}

extern "C" void Init(void* argument)
{
    (void)argument;

    flags_create();
    Controller::app_controller_receive_init();
    Device::app_device_init();
    APP_Clamp_BeforeUpdate();
    APP_Arm_BeforeUpdate();

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    osThreadExit();
}
