/* 初始化代码 */

/**
 * @file    app.h
 * @author  rediduck
 * @date    2026-05-1
 */

#include "cmsis_os2.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "main.h"

#include "device.hpp"
#include "controller.hpp"
#include "flags.hpp"
#include "clamp.hpp"
#include "arm.hpp"

osThreadId_t         softTIMHandle;
const osThreadAttr_t softTIM_attributes = {
    .name       = "softTIM",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityRealtime7,
};

////////////////////////一些回调处理函数////////////////////////

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
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

/////////////////////////////////////////////////////////////

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    flags_create();
    Controller::app_controller_receive_init();
    Device::app_device_init();
    APP_Clamp_BeforeUpdate();
    APP_Arm_BeforeUpdate();

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    // Device::waitAllConnected();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
