#include "arm.hpp"

#include "device.hpp"
#include "main.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"

#include <cmath>
#include <cstdint>

namespace Arm
{

// ================================= 气泵控制相关定义与函数
// ==========================================
typedef struct
{
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
    GPIO_TypeDef*      valve_port;
    GPIO_TypeDef*      pump_port;
    uint16_t           valve_pin;
    uint16_t           pump_pin;
    uint8_t            invert;
} Pump_Config_t;

typedef struct
{
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
    GPIO_TypeDef*      valve_port;
    GPIO_TypeDef*      pump_port;
    uint16_t           valve_pin;
    uint16_t           pump_pin;
    uint8_t            invert;
} Pump_t;

static void Pump_ValveOn(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->valve_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_SET);
}

static void Pump_ValveOff(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->valve_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_RESET);
}

static void Pump_RelayOn(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->pump_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_SET);
}

static void Pump_RelayOff(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->pump_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_RESET);
}

static void Pump_Catch(Pump_t* hpump, uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    Pump_ValveOff(hpump);
    Pump_RelayOn(hpump);
}

static void Pump_Release(Pump_t* hpump, uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    Pump_ValveOn(hpump);
    Pump_RelayOff(hpump);
}

static void Pump_Init(Pump_t* hpump, const Pump_Config_t* config)
{
    if (hpump == nullptr || config == nullptr)
    {
        return;
    }

    hpump->htim       = config->htim;
    hpump->channel    = config->channel;
    hpump->valve_port = config->valve_port;
    hpump->pump_port  = config->pump_port;
    hpump->pump_pin   = config->pump_pin;
    hpump->valve_pin  = config->valve_pin;
    hpump->invert     = config->invert;

    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_RESET);
}

// ================================= 机械臂控制参数 =======================================
#define ARM_RESET_ANGLE          0.0f  // 初始位置的值设定
#define ARM_CATCH_PUSH_ANGLE     80.0f // 初始位置为收在最里面
#define ARM_CATCH_PUSH_ANGLE_MAX 90.0f
#define ARM_CATCH_HEIGHT_LOW     -1300.0f // 初始位置为顶端，这个值是从顶端往下数的，负值
#define ARM_CATCH_HEIGHT_MID     -707.0f
#define ARM_CATCH_HEIGHT_HIGH    0.0f
#define ARM_RELEASE_HEIGHT       -150.0f
#define ARM_ROTATE_ANGLE         -300.0f // 这个是从收回转到取物是-360，
#define ARM_ROTATE_RELEASE_ANGLE 60.0f // 这个是从收回转到放物时的旋转角度，60是先放物的角度

#define ARM_AUTO_WAIT_HEIGHT_MS                      500U
#define ARM_AUTO_WAIT_ROTATE_MS                      500U
#define ARM_AUTO_WAIT_PUMP_ON_MS                     50U
#define ARM_AUTO_WAIT_PUSH_MS                        500U
#define ARM_AUTO_WAIT_BACK_MS                        500U
#define ARM_AUTO_WAIT_RELEASE_HEIGHT_MS              1500U
#define ARM_AUTO_WAIT_ROTATE_BACK_MS                 800U
#define ARM_AUTO_WAIT_ROTATE_BACK_AND_RELEASE_HEIGHT 2000U
#define ARM_AUTO_WAIT_RELEASE_MS                     500U
#define ARM_AUTO_WAIT_RESET_POS_MS                   2000U
#define ARM_AUTO_RETRACT_PUSH_ANGLE                  20.0f
#define ARM_AUTO_RETREAT_MIN_TIME_MS                 300U
#define ARM_AUTO_RETREAT_MAX_TIME_MS                 2500U
#define ARM_RAISEANDLOWER_POS_DEADZONE_DEG           10.0f

#define PUMP_VALVE_GPIO_Port GPIOC
#define PUMP_VALVE_Pin       GPIO_PIN_0
#define PUMP_RELAY_GPIO_Port GPIOC
#define PUMP_RELAY_Pin       GPIO_PIN_1

using Motor_PosCtrl_t = controllers::MotorPosController;
using Motor_VelCtrl_t = controllers::MotorVelController;
using namespace Device::motor;

bool Is_raiseandlower_motor_init = false;
bool Is_rotate_motor_out         = false;

float arm_vel_out         = 0;
float arm_vel_out_last    = 0;
float arm_vel_rotate      = 0;
float arm_vel_rotate_last = 0;
float arm_vel_height      = 0;
float arm_vel_height_last = 0;
float arm_pos_out         = 0;
float arm_pos_rotate      = 0;
float arm_pos_height      = 0;

enum AutoCatchState
{
    AUTO_CATCH_IDLE = 0,
    AUTO_CATCH_ROTATE,
    AUTO_CATCH_HEIGHT_AND_PUMP,
    AUTO_CATCH_PUSH_OUT,
    AUTO_CATCH_BACK,
    AUTO_HEIGHT_BACK,
    AUTO_CATCH_GO_RELEASE_HEIGHT_AND_ROTATE,
    AUTO_CATCH_ROTATE_AND_GO_RELEASE_HEIGHT,
    AUTO_CATCH_RELEASE,
    AUTO_RESET_POS
};

static volatile AutoCatchState    g_auto_catch_state          = AUTO_CATCH_IDLE;
static volatile uint32_t          g_auto_catch_state_start_ms = 0;
static volatile ArmAutoCatchLevel g_auto_catch_target_height  = ARM_AUTO_CATCH_HIGH;
static volatile bool              g_auto_catch_continue       = false;

// ================================= 机械臂控制相关变量与函数声明
// ===================================
osTimerId_t arm_timHandle = nullptr;
// float arm_pos_height = ARM_RESET_ANGLE;//ARM_CATCH_HEIGHT_LOW;

osThreadId_t         ArmHandle      = nullptr;
const osThreadAttr_t arm_attributes = {
    .name       = "arm",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal1,
};

Motor_PosCtrl_t* pos_rotate_motor        = nullptr;
Motor_PosCtrl_t* pos_raiseandlower_motor = nullptr;
Motor_PosCtrl_t* pos_catch_motor         = nullptr;

Motor_VelCtrl_t* vel_rotate_motor        = nullptr;
Motor_VelCtrl_t* vel_raiseandlower_motor = nullptr;
Motor_VelCtrl_t* vel_catch_motor         = nullptr;

Pump_Config_t pump_config = {
    .htim       = nullptr,
    .channel    = TIM_CHANNEL_3,
    .valve_port = PUMP_VALVE_GPIO_Port,
    .pump_port  = PUMP_RELAY_GPIO_Port,
    .valve_pin  = PUMP_VALVE_Pin,
    .pump_pin   = PUMP_RELAY_Pin,
    .invert     = 1,
};

static Pump_t pump;

static void Arm_raiseandlower_set_pos_ref(float position)
{
    arm_pos_height = position;
    if (pos_raiseandlower_motor != nullptr)
    {
        pos_raiseandlower_motor->setRef(position);
    }
}

static void Arm_raiseandlower_reset(float vel)
{
    vel_raiseandlower_motor->enable();
    vel_raiseandlower_motor->setRef(vel);
    Is_raiseandlower_motor_init = true;

    while (std::fabs(vel_raiseandlower_motor->getPID().getOutput()) < 13000.0f)
    {
        osDelay(1);
    }
    // 检测输出稳定性（需保持500ms以上）
    uint32_t       stable_time_start = HAL_GetTick();
    const uint32_t STABLE_THRESHOLD  = 500;

    while (Is_raiseandlower_motor_init)
    {
        if (std::fabs(vel_raiseandlower_motor->getPID().getOutput()) >= 10000.0f)
        {
            // 输出保持在目标以上
            if (HAL_GetTick() - stable_time_start >= STABLE_THRESHOLD)
            {
                Is_raiseandlower_motor_init = false;
            }
        }
        else
        {
            // 输出下降到目标以下，重置计时器
            stable_time_start = HAL_GetTick();
        }
        osDelay(1);
    }
    Device::motor::raiseandlower_motor->resetAngle();
    vel_raiseandlower_motor->setRef(0.0f);
}

// 将所有输出使能/位置设定重置到初始状态。
static void Arm_output_reset()
{
    vel_catch_motor->disable();
    pos_catch_motor->disable();

    vel_rotate_motor->disable();
    pos_rotate_motor->disable();

    vel_raiseandlower_motor->disable();
    pos_raiseandlower_motor->disable();

    arm_vel_out    = 0;
    arm_vel_rotate = 0;
    arm_vel_height = 0;

    arm_vel_out_last    = 0;
    arm_vel_rotate_last = 0;
    arm_vel_height_last = 0;

    arm_pos_out    = 0;
    arm_pos_rotate = 0;
    arm_pos_height = 0;

    Is_rotate_motor_out = false;
}

// 电机控制中断节拍：更新所有位置/速度控制器。
void Arm_TIM_Callback(void)
{
    pos_raiseandlower_motor->update();
    pos_catch_motor->update();
    pos_rotate_motor->update();
    vel_raiseandlower_motor->update();
    vel_rotate_motor->update();
    vel_catch_motor->update();

    if (pos_raiseandlower_motor != nullptr && pos_raiseandlower_motor->enabled())
    {
        motors::IMotor* motor = pos_raiseandlower_motor->getMotor();
        if (motor != nullptr)
        {
            const float pos_error = arm_pos_height - motor->getAngle();
            if (std::fabs(pos_error) <= ARM_RAISEANDLOWER_POS_DEADZONE_DEG)
            {
                pos_raiseandlower_motor->setRef(motor->getAngle());
                pos_raiseandlower_motor->update();
            }
        }
    }
}

// ================================ 机械臂自动抓取状态机相关函数实现
// ===================================== 判断自动抓取当前步骤是否超时。
static bool AutoStepTimeout(uint32_t wait_ms, uint32_t now_ms)
{
    return (uint32_t)(now_ms - g_auto_catch_state_start_ms) >= wait_ms;
}

// 切换自动抓取状态并记录进入时间。
static void AutoCatchEnterState(AutoCatchState state, uint32_t now_ms)
{
    g_auto_catch_state          = state;
    g_auto_catch_state_start_ms = now_ms;
    if (state == AUTO_CATCH_BACK)
    {
        g_auto_catch_continue = false;
    }
}

// 返回自动抓取流程是否在运行。
bool Arm_AutoCatchBusy()
{
    return g_auto_catch_state != AUTO_CATCH_IDLE;
}

// 紧急终止，中止自动抓取但保持吸泵当前状态；停止所有电机。
void Arm_AutoCatchAbortKeepPump()
{
    if (g_auto_catch_state == AUTO_CATCH_IDLE)
    {
        return;
    }

    if (pos_catch_motor)
    {
        pos_catch_motor->disable();
    }
    if (vel_catch_motor)
    {
        vel_catch_motor->disable();
    }
    if (pos_rotate_motor)
    {
        pos_rotate_motor->disable();
    }
    if (vel_rotate_motor)
    {
        vel_rotate_motor->disable();
    }
    if (pos_raiseandlower_motor)
    {
        pos_raiseandlower_motor->disable();
    }
    if (vel_raiseandlower_motor)
    {
        vel_raiseandlower_motor->disable();
    }

    arm_vel_out         = 0;
    arm_vel_rotate      = 0;
    arm_vel_height      = 0;
    arm_vel_out_last    = 0;
    arm_vel_rotate_last = 0;
    arm_vel_height_last = 0;
    g_auto_catch_state  = AUTO_CATCH_IDLE;
}

// ================================ 遥控器接口函数和参数实现
// ============================================

// 下面的三个函数都是通过按键触发的

// 启动自动抓取状态机并清零手动速度指令,通过这个函数可以从外部触发自动抓取流程，参数指定抓取的高度档位。
bool Arm_AutoCatchStart(ArmAutoCatchLevel level)
{
    if (g_auto_catch_state != AUTO_CATCH_IDLE)
    {
        return false;
    }

    switch (level)
    {
    case ARM_AUTO_CATCH_LOW:
        g_auto_catch_target_height = ARM_AUTO_CATCH_LOW;
        break;
    case ARM_AUTO_CATCH_MID:
        g_auto_catch_target_height = ARM_AUTO_CATCH_MID;
        break;
    case ARM_AUTO_CATCH_HIGH:
        g_auto_catch_target_height = ARM_AUTO_CATCH_HIGH;
        break;
    default:
        return false;
    }

    arm_vel_out           = 0;
    arm_vel_rotate        = 0;
    arm_vel_height        = 0;
    arm_vel_out_last      = 0;
    arm_vel_rotate_last   = 0;
    arm_vel_height_last   = 0;
    g_auto_catch_continue = false;

    AutoCatchEnterState(AUTO_CATCH_ROTATE, HAL_GetTick());
    return true;
}

void Arm_AutoCatchContinue()
{
    if (g_auto_catch_state == AUTO_HEIGHT_BACK)
    {
        g_auto_catch_continue = true;
    }
}

void Arm_Rotate_Out(bool enable)
{
    if (!enable)
    {
        return;
    }
    if (pos_rotate_motor == nullptr || vel_rotate_motor == nullptr)
    {
        return;
    }
    if (Is_rotate_motor_out)
    {
        return;
    }

    vel_rotate_motor->disable();
    pos_rotate_motor->enable();
    pos_rotate_motor->setRef(ARM_ROTATE_ANGLE);
    Is_rotate_motor_out = true;
}

void Arm_Rotate_Back(bool enable)
{
    if (!enable)
    {
        return;
    }
    if (pos_rotate_motor == nullptr || vel_rotate_motor == nullptr)
    {
        return;
    }
    if (!Is_rotate_motor_out)
    {
        return;
    }

    vel_rotate_motor->disable();
    pos_rotate_motor->enable();
    pos_rotate_motor->setRef(ARM_RESET_ANGLE);
    Is_rotate_motor_out = false;
}

// 切换吸泵：吸取 <-> 释放
void Arm_Pump_Toggle(void)
{
    static uint8_t pump_state = 0;
    if (pump_state == 0)
    {
        pump_state = 1;
        Pump_Catch(&pump, 1);
    }
    else
    {
        pump_state = 0;
        Pump_Release(&pump, 1);
    }
}

// ================================ 预留的应用层钩子实现(包括主循环与初始化) ================================
static void Arm_softTIM(void* argument)
{
    (void)argument;

    const uint32_t now_ms = HAL_GetTick();

    if (g_auto_catch_state == AUTO_CATCH_IDLE)
    {
        pos_catch_motor->enable();
        pos_catch_motor->setRef(ARM_RESET_ANGLE);
    }

    if (g_auto_catch_state != AUTO_CATCH_IDLE)
    {
        switch (g_auto_catch_state)
        {
        case AUTO_CATCH_ROTATE: // 开始自动抓取流程，先旋转出来，防止自身机构卡住
            // 如果已经在旋转位置了（可能是从放物位置转回来的），就直接进入下一个状态
            if (Is_rotate_motor_out)
            {
                AutoCatchEnterState(AUTO_CATCH_HEIGHT_AND_PUMP, now_ms);
            }
            vel_rotate_motor->disable();
            pos_rotate_motor->enable();
            pos_rotate_motor->setRef(ARM_ROTATE_ANGLE);
            if (AutoStepTimeout(ARM_AUTO_WAIT_ROTATE_MS, now_ms))
            {
                AutoCatchEnterState(AUTO_CATCH_HEIGHT_AND_PUMP, now_ms);
            }
            break;

        case AUTO_CATCH_HEIGHT_AND_PUMP: // 旋转之后进行高度调整和吸泵
            vel_raiseandlower_motor->disable();
            pos_raiseandlower_motor->enable();
            Pump_Catch(&pump, 1);
            switch (g_auto_catch_target_height)
            {
            case ARM_AUTO_CATCH_LOW:
                Arm_raiseandlower_set_pos_ref(ARM_CATCH_HEIGHT_LOW);
                break;
            case ARM_AUTO_CATCH_MID:
                Arm_raiseandlower_set_pos_ref(ARM_CATCH_HEIGHT_MID);
                break;
            case ARM_AUTO_CATCH_HIGH:
                Arm_raiseandlower_set_pos_ref(ARM_CATCH_HEIGHT_HIGH);
                break;
            }
            if (AutoStepTimeout(ARM_AUTO_WAIT_HEIGHT_MS, now_ms))
            {
                AutoCatchEnterState(AUTO_CATCH_PUSH_OUT, now_ms);
            }
            break;

        case AUTO_CATCH_PUSH_OUT: // 向前推出吸取卷轴，注意对不同高度的卷轴之后先旋转还是先去释放位置顺序不同
            vel_catch_motor->disable();
            pos_catch_motor->enable();
            pos_catch_motor->setRef(ARM_CATCH_PUSH_ANGLE);
            if (AutoStepTimeout(ARM_AUTO_WAIT_PUSH_MS, now_ms))
            {
                AutoCatchEnterState(AUTO_CATCH_BACK, now_ms);
            }
            break;

        case AUTO_CATCH_BACK: // 推出后把机械臂收回，准备去放物位置
            vel_catch_motor->disable();
            pos_catch_motor->enable();
            pos_catch_motor->setRef(ARM_AUTO_RETRACT_PUSH_ANGLE);
            if (AutoStepTimeout(ARM_AUTO_WAIT_BACK_MS, now_ms))
            {
                AutoCatchEnterState(AUTO_HEIGHT_BACK, now_ms);
            }
            break;

        case AUTO_HEIGHT_BACK: //先回初始高度，方便开车
                vel_raiseandlower_motor->disable();
                pos_raiseandlower_motor->enable();
                Arm_raiseandlower_set_pos_ref(ARM_CATCH_HEIGHT_HIGH);
                if (AutoStepTimeout(ARM_AUTO_WAIT_BACK_MS, now_ms) && g_auto_catch_continue)
                {
                    g_auto_catch_continue = false;
                    AutoCatchEnterState(AUTO_CATCH_ROTATE_AND_GO_RELEASE_HEIGHT, now_ms);
                }
                break;

        case AUTO_CATCH_ROTATE_AND_GO_RELEASE_HEIGHT: // 先旋转再去释放高度的逻辑
            vel_rotate_motor->disable();
            pos_rotate_motor->enable();
            pos_rotate_motor->setRef(ARM_ROTATE_RELEASE_ANGLE);
            if (AutoStepTimeout(ARM_AUTO_WAIT_ROTATE_BACK_MS, now_ms))
            {
                vel_raiseandlower_motor->disable();
                pos_raiseandlower_motor->enable();
                Arm_raiseandlower_set_pos_ref(ARM_RELEASE_HEIGHT);
                if (AutoStepTimeout(ARM_AUTO_WAIT_ROTATE_BACK_AND_RELEASE_HEIGHT, now_ms))
                {
                    AutoCatchEnterState(AUTO_CATCH_RELEASE, now_ms);
                }
            }
            break;

        case AUTO_CATCH_RELEASE: // 到达释放高度后关泵放下物体，并保持一段时间后结束流程

            vel_catch_motor->disable();
            pos_catch_motor->enable();
            pos_catch_motor->setRef(ARM_AUTO_RETRACT_PUSH_ANGLE);
            Pump_Release(&pump, 1);
            if (AutoStepTimeout(ARM_AUTO_WAIT_RELEASE_MS, now_ms))
            {
                arm_vel_out         = 0;
                arm_vel_rotate      = 0;
                arm_vel_height      = 0;
                arm_vel_out_last    = 0;
                arm_vel_rotate_last = 0;
                arm_vel_height_last = 0;
                g_auto_catch_state  = AUTO_RESET_POS;
            }
            break;

        case AUTO_RESET_POS: // 释放完成后把机械臂转回最高位置并转出来，准备下一次操作

            vel_catch_motor->disable();
            pos_catch_motor->enable();
            pos_catch_motor->setRef(ARM_RESET_ANGLE);
            vel_rotate_motor->disable();
            pos_rotate_motor->enable();
            pos_rotate_motor->setRef(ARM_ROTATE_ANGLE);
            Is_rotate_motor_out = true;
            vel_raiseandlower_motor->disable();
            pos_raiseandlower_motor->enable();
            Arm_raiseandlower_set_pos_ref(ARM_RESET_ANGLE);
            if (AutoStepTimeout(ARM_AUTO_WAIT_RESET_POS_MS, now_ms))
            {
                AutoCatchEnterState(AUTO_CATCH_IDLE, now_ms);
            }
            break;

        case AUTO_CATCH_IDLE:
        default:
            break;
        }
        return;
    }
}

// 将外部速度指令转换为控制器使能/失能动作。
static void Arm_Contrl_Task(void* argument)
{
    (void)argument;
    Arm_raiseandlower_reset(30.0f);
    if (raiseandlower_motor != nullptr)
    {
        raiseandlower_motor->resetAngle();
    }
    vel_raiseandlower_motor->getPID().reset();
    for (;;)
    {
        osDelay(10);
    }
}

// 初始化吸泵、控制器与 RTOS 钩子。

namespace
{
inline constexpr Motor_VelCtrl_t::Config arm_catch_vel_cfg{
    .pid = { .Kp = 25.0f, .Ki = 0.15f, .Kd = 20.0f, .abs_output_max = 5000.0f },
};
inline constexpr Motor_VelCtrl_t::Config arm_rotate_vel_cfg{
    .pid = { .Kp = 450.0f, .Ki = 1.0f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};
inline constexpr Motor_VelCtrl_t::Config arm_raiseandlower_vel_cfg{
    .pid = { .Kp = 100.0f, .Ki = 0.8f, .Kd = 1.0f, .abs_output_max = 14000.0f },
};
inline constexpr Motor_PosCtrl_t::Config arm_catch_pos_cfg{
    .position_pid       = { .Kp = 3.0f, .Ki = 0.1f, .Kd = 0.2f, .abs_output_max = 800.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.2f, .Kd = 0.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10,
};
inline constexpr Motor_PosCtrl_t::Config arm_rotate_pos_cfg{
    .position_pid       = { .Kp = 3.0f, .Ki = 0.00f, .Kd = 2.0f, .abs_output_max = 70.0f },
    .velocity_pid       = { .Kp = 450.0f, .Ki = 1.0f, .Kd = 0.0f, .abs_output_max = 12000.0f },
    .pos_vel_freq_ratio = 10,
};
inline constexpr Motor_PosCtrl_t::Config arm_raiseandlower_pos_cfg{
    .position_pid       = { .Kp = 1.6f, .Ki = 0.0f, .Kd = 0.6f, .abs_output_max = 300.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 5.0f, .Kd = 0.5f, .abs_output_max = 14000.0f },
    .pos_vel_freq_ratio = 1,
};
} // namespace

void Arm_Init(void)
{
    (void)ARM_CATCH_PUSH_ANGLE;
    (void)ARM_CATCH_PUSH_ANGLE_MAX;
    (void)ARM_RELEASE_HEIGHT;

    Pump_Init(&pump, &pump_config);

    vel_catch_motor         = new Motor_VelCtrl_t(catch_motor, arm_catch_vel_cfg);
    pos_catch_motor         = new Motor_PosCtrl_t(catch_motor, arm_catch_pos_cfg);
    vel_rotate_motor        = new Motor_VelCtrl_t(rotate_motor, arm_rotate_vel_cfg);
    pos_rotate_motor        = new Motor_PosCtrl_t(rotate_motor, arm_rotate_pos_cfg);
    vel_raiseandlower_motor = new Motor_VelCtrl_t(raiseandlower_motor, arm_raiseandlower_vel_cfg);
    pos_raiseandlower_motor = new Motor_PosCtrl_t(raiseandlower_motor, arm_raiseandlower_pos_cfg);

    Arm_output_reset();
    pos_rotate_motor->enable();
    pos_rotate_motor->setRef(ARM_RESET_ANGLE);
    ArmHandle     = osThreadNew(Arm_Contrl_Task, NULL, &arm_attributes);
    arm_timHandle = osTimerNew(Arm_softTIM, osTimerPeriodic, NULL, NULL);
    osTimerStart(arm_timHandle, 10);
}

// 应用层钩子：初始化机械臂子系统。
void app_arm_init()
{
    Arm_Init();
}

// 应用层钩子：1 kHz 控制器更新节拍。
void app_arm_update_1kHz()
{
    Arm_TIM_Callback();
}

} // namespace Arm
