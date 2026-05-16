#include "push.hpp"
#include "device.hpp"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "config.hpp"
#include "flags.hpp"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

namespace Push
{
using MotorPosController = controllers::MotorPosController;
using MotorVelController = controllers::MotorVelController;

static ControlMode push_mode  = ControlMode::Pos;
float              target_vel = 0.0f;
static float       target_pos = 0.0f;

const osThreadAttr_t push_attributes = {
    .name       = "push",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal1,
};

MotorPosController* push_pos = nullptr;
MotorVelController* push_vel = nullptr;

static bool is_zero_velocity(const float velocity)
{
    return std::fabs(velocity) <= zero_velocity_epsilon;
}

static void hold_axis_position(MotorPosController* position_controller,
                               MotorVelController* velocity_controller,
                               ControlMode&        axis_mode,
                               const float         commanded_velocity,
                               float&              target_position)
{
    if (is_zero_velocity(commanded_velocity))
    {
        if (axis_mode != ControlMode::Pos)
        {
            target_position = position_controller->getMotor()->getAngle();
            axis_mode       = ControlMode::Pos;
        }
    }
    else
    {
        axis_mode = ControlMode::Vel;
    }
}

static void update_axis_controller(MotorPosController* position_controller,
                                   MotorVelController* velocity_controller,
                                   const ControlMode   axis_mode,
                                   const float         velocity_ref,
                                   const float         position_ref)
{
    switch (axis_mode)
    {
    case ControlMode::Pos:
        velocity_controller->disable();
        position_controller->enable();
        position_controller->setRef(position_ref);
        position_controller->update();
        break;

    case ControlMode::Vel:
        position_controller->disable();
        velocity_controller->enable();
        velocity_controller->setRef(velocity_ref);
        velocity_controller->update();
        break;
    }
}

void PushControl(void* argument)
{
    if (button_state & PUSH_MASK)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2500);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
    }
}

static void push_init(void)
{
    push_pos = new controllers::MotorPosController(Device::motor::motor_push,
                                                   AppConfig::Clamp::kM2006PosConfig);
    push_vel = new controllers::MotorVelController(
            Device::motor::motor_push,
            MotorVelController::Config{ .pid = AppConfig::Clamp::kM2006PosConfig.velocity_pid });
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2500);
}

static void push_control_init(void)
{
    osThreadNew(PushControl, nullptr, &push_attributes);
}

static void push_timer_callback()
{
    hold_axis_position(push_pos, push_vel, push_mode, target_vel, target_pos);
    update_axis_controller(push_pos, push_vel, push_mode, target_vel, target_pos);
}

void app_push_init()
{
    push_init();
    push_control_init();
}

void app_push_update_1kHz()
{
    push_timer_callback();
}
} // namespace Push