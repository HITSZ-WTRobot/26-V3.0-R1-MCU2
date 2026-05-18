#include "clamp.hpp"

#include "cmsis_os2.h"
#include "config.hpp"
#include "device.hpp"
#include "flags.hpp"
#include "main.h"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"

#include <cmath>

namespace Clamp
{
using namespace Device::motor;

float        clamp_vel_out  = 0.0f;
float        clamp_vel_yaw  = 0.0f;
float        clamp_vel_roll = 0.0f;
ResetProcess reset_status   = ResetProcess::Unknown;
clamp_status clamp_state    = clamp_status::Init;
namespace
{

using MotorPosController = controllers::MotorPosController;
using MotorVelController = controllers::MotorVelController;

inline constexpr uint32_t control_task_delay_ms    = 100U;    // 控制线程周期
inline constexpr float    out_reset_seek_velocity  = -100.0f; // out复位速度
inline constexpr float    out_reset_lock_threshold = 1000.0f; // 电流阈值
inline constexpr uint32_t out_reset_hold_ms        = 500U;    // 达到阈值后保持时间
inline constexpr float    yaw_reset_seek_velocity  = -50.0f;  // yaw复位速度
inline constexpr float    yaw_reset_lock_threshold = 1000.0f; // yaw电流阈值
inline constexpr uint32_t yaw_reset_hold_ms        = 500U;    // 达到阈值后保持时间
inline constexpr float    yaw_reset_hold_position  = 90.0f;   // yaw复位完成后保持位置
inline constexpr float    catch_closed_angle       = 150.0f;  // 夹爪闭合位置
inline constexpr float    zero_velocity_epsilon    = 1e-1f;   // 判断速度为0阈值

MotorPosController* clamp_out_pos   = nullptr;
MotorPosController* clamp_yaw_pos   = nullptr;
MotorPosController* clamp_roll_pos  = nullptr;
MotorPosController* clamp_catch_pos = nullptr;

MotorVelController* clamp_out_vel  = nullptr;
MotorVelController* clamp_yaw_vel  = nullptr;
MotorVelController* clamp_roll_vel = nullptr;

ControlMode clamp_out_mode  = ControlMode::Vel;
ControlMode clamp_roll_mode = ControlMode::Vel;
ControlMode clamp_yaw_mode  = ControlMode::Vel;

bool  control_reset = false;
float target_out    = 0.0f;
float target_yaw    = 0.0f;
float target_roll   = 0.0f;
float catch_angle   = 0.0f;

const osThreadAttr_t clamp_attributes = {
    .name       = "clamp",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal1,
};

bool is_zero_velocity(const float velocity)
{
    return std::fabs(velocity) <= zero_velocity_epsilon;
}

void hold_axis_position(MotorPosController* position_controller,
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
        position_controller->disable();
        velocity_controller->enable();
    }
}

void update_axis_controller(MotorPosController* position_controller,
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

void clamp_timer_callback()
{
    hold_axis_position(clamp_out_pos, clamp_out_vel, clamp_out_mode, clamp_vel_out, target_out);
    hold_axis_position(
            clamp_roll_pos, clamp_roll_vel, clamp_roll_mode, clamp_vel_roll, target_roll);
    hold_axis_position(clamp_yaw_pos, clamp_yaw_vel, clamp_yaw_mode, clamp_vel_yaw, target_yaw);

    update_axis_controller(clamp_out_pos, clamp_out_vel, clamp_out_mode, clamp_vel_out, target_out);
    update_axis_controller(
            clamp_roll_pos, clamp_roll_vel, clamp_roll_mode, clamp_vel_roll, target_roll);
    update_axis_controller(clamp_yaw_pos, clamp_yaw_vel, clamp_yaw_mode, clamp_vel_yaw, target_yaw);

    clamp_catch_pos->setRef(catch_angle);
    clamp_catch_pos->update();
}

void wait_axis_stall(MotorVelController* velocity_controller,
                     const float         lock_threshold,
                     const uint32_t      hold_ms)
{
    reset_status = ResetProcess::Processing;

    while (std::fabs(velocity_controller->getPID().getOutput()) < lock_threshold)
    {
        osDelay(1);
    }

    for (;;)
    {
        while (std::fabs(velocity_controller->getPID().getOutput()) < lock_threshold)
        {
            osDelay(1);
        }

        const uint32_t start_tick = HAL_GetTick();
        while (std::fabs(velocity_controller->getPID().getOutput()) >= lock_threshold)
        {
            reset_status = ResetProcess::Wait;
            if ((HAL_GetTick() - start_tick) >= hold_ms)
            {
                return;
            }
            osDelay(1);
        }
    }
}

void reset_clamp_out_axis()
{
    clamp_out_mode = ControlMode::Vel;
    clamp_vel_out  = out_reset_seek_velocity;
    wait_axis_stall(clamp_out_vel, out_reset_lock_threshold, out_reset_hold_ms);
}

void finish_clamp_out_reset()
{
    clamp_vel_out = 0.0f;
    if (motor_clamp_out != nullptr)
    {
        motor_clamp_out->resetAngle();
    }
    clamp_out_vel->getPID().reset();
    target_out     = 50.0f;
    clamp_out_mode = ControlMode::Pos;
}

void reset_clamp_yaw_axis()
{
    clamp_yaw_mode = ControlMode::Vel;
    clamp_vel_yaw  = yaw_reset_seek_velocity;
    wait_axis_stall(clamp_yaw_vel, yaw_reset_lock_threshold, yaw_reset_hold_ms);
}

void finish_clamp_yaw_reset()
{
    clamp_vel_yaw = 0.0f;
    if (motor_clamp_yaw != nullptr)
    {
        motor_clamp_yaw->resetAngle();
    }
    clamp_yaw_vel->getPID().reset();
    target_yaw     = yaw_reset_hold_position;
    clamp_yaw_mode = ControlMode::Pos;
}

} // namespace

static void ClampControl(void* argument)
{
    (void)argument;

    for (;;)
    {
        if (control_reset)
        {
            reset_status   = ResetProcess::Start;
            clamp_vel_roll = 0.0f;

            reset_clamp_out_axis();
            finish_clamp_out_reset();

            reset_status = ResetProcess::Processing;
            reset_clamp_yaw_axis();
            finish_clamp_yaw_reset();

            reset_status  = ResetProcess::Success;
            control_reset = false;
        }
        if (reset_status == ResetProcess::Success)
        {
            if (((button_state & DIP4_MASK) == DIP4_CASE0) && (clamp_state != clamp_status::Out))
            {
                clamp_state = clamp_status::Out;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
                osDelay(500);
                target_roll = 0.0f;
                osDelay(500);
                catch_angle = 0.0f;
                osDelay(500);
            }
            else if (((button_state & DIP4_MASK) == DIP4_CASE1) &&
                     (clamp_state != clamp_status::In))
            {
                clamp_state = clamp_status::In;
                catch_angle = 200;
                osDelay(500);
                target_roll = -270.0f;
                osDelay(1000);
                target_yaw = 90;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            }
            else if (((button_state & DIP4_MASK) == DIP4_CASE2) &&
                     (clamp_state != clamp_status::Dock))
            {
                target_yaw = 0.0f;
                osDelay(1000);
                target_roll = -270.0f;
                clamp_state = clamp_status::Dock;
            }
        }
        osDelay(control_task_delay_ms);
    }
}

static void clamp_init(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    clamp_out_vel = new MotorVelController(
            motor_clamp_out,
            MotorVelController::Config{ .pid = AppConfig::Clamp::kM2006PosConfig.velocity_pid });
    clamp_yaw_vel = new MotorVelController(
            motor_clamp_yaw,
            MotorVelController::Config{ .pid = AppConfig::Clamp::kM3508PosConfig.velocity_pid });
    clamp_roll_vel = new MotorVelController(
            motor_clamp_roll,
            MotorVelController::Config{ .pid = AppConfig::Clamp::kM2006PosConfig.velocity_pid });

    clamp_out_pos   = new MotorPosController(motor_clamp_out, AppConfig::Clamp::kM2006PosConfig);
    clamp_roll_pos  = new MotorPosController(motor_clamp_roll, AppConfig::Clamp::kM2006PosConfig);
    clamp_yaw_pos   = new MotorPosController(motor_clamp_yaw, AppConfig::Clamp::kM3508PosConfig);
    clamp_catch_pos = new MotorPosController(motor_clamp_catch,
                                             AppConfig::Clamp::kM2006PosConfig_low);

    clamp_out_pos->disable();
    clamp_roll_pos->disable();
    clamp_yaw_pos->disable();
    clamp_out_vel->disable();
    clamp_roll_vel->disable();
    clamp_yaw_vel->disable();
    clamp_catch_pos->enable();
}

static void clamp_control_init(void)
{
    osThreadNew(ClampControl, nullptr, &clamp_attributes);
    control_reset = 1;
}

void app_clamp_init()
{
    clamp_init();
    clamp_control_init();
}

void app_clamp_update_1kHz()
{
    clamp_timer_callback();
}
} // namespace Clamp
