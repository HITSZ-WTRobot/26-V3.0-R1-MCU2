#include "clamp.hpp"

#include "cmsis_os2.h"
#include "config.hpp"
#include "device.hpp"
#include "flags.hpp"
#include "main.h"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"

#include <cmath>

using namespace Device::motor;

float        clamp_vel_out  = 0.0f;
float        clamp_vel_yaw  = 0.0f;
float        clamp_vel_roll = 0.0f;
ResetProcess reset_status   = ResetProcess::Unknown;

namespace
{

using MotorPosController = controllers::MotorPosController;
using MotorVelController = controllers::MotorVelController;

inline constexpr MotorVelController::Config clamp_out_vel_controller_config{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr MotorVelController::Config clamp_yaw_vel_controller_config{
    .pid = { .Kp = 45.0f, .Ki = 0.15f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr MotorVelController::Config clamp_roll_vel_controller_config{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 8000.0f },
};

inline constexpr MotorVelController::Config clamp_catch_vel_controller_config{
    .pid = { .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 4500.0f },
};

inline constexpr MotorPosController::Config clamp_out_pos_controller_config{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr MotorPosController::Config clamp_roll_pos_controller_config{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr MotorPosController::Config clamp_yaw_pos_controller_config{
    .position_pid       = { .Kp = 80.0f, .Ki = 1.0f, .Kd = 0.0f, .abs_output_max = 2000.0f },
    .velocity_pid       = { .Kp = 12.0f, .Ki = 0.20f, .Kd = 5.0f, .abs_output_max = 8000.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr MotorPosController::Config clamp_catch_pos_controller_config{
    .position_pid       = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400.0f },
    .velocity_pid       = { .Kp = 500.0f, .Ki = 0.10f, .Kd = 0.0f, .abs_output_max = 4500.0f },
    .pos_vel_freq_ratio = 10U,
};

inline constexpr uint32_t soft_timer_period_ms      = 20U;
inline constexpr uint32_t control_task_delay_ms     = 100U;
inline constexpr float    out_reset_seek_velocity   = 100.0f;
inline constexpr float    out_reset_lock_threshold  = 1500.0f;
inline constexpr uint32_t out_reset_hold_ms         = 500U;
inline constexpr float    yaw_reset_seek_velocity   = 50.0f;
inline constexpr float    yaw_reset_lock_threshold  = 1500.0f;
inline constexpr uint32_t yaw_reset_hold_ms         = 500U;
inline constexpr float    yaw_reset_hold_position   = 0.0f;
inline constexpr float    catch_closed_angle        = 150.0f;
inline constexpr float    zero_velocity_epsilon     = 1e-4f;
inline constexpr uint32_t clamp_reset_event_flag    = 0x00000080U;
inline constexpr uint32_t clamp_catch_toggle_flag   = 0x00000040U;
inline constexpr uint32_t event_wait_mask_base      = 0xFF000000U;

MotorPosController* clamp_out_pos   = nullptr;
MotorPosController* clamp_yaw_pos   = nullptr;
MotorPosController* clamp_roll_pos  = nullptr;
MotorPosController* clamp_catch_pos = nullptr;

MotorVelController* clamp_out_vel_controller   = nullptr;
MotorVelController* clamp_yaw_vel_controller   = nullptr;
MotorVelController* clamp_roll_vel_controller  = nullptr;
MotorVelController* clamp_catch_vel_controller = nullptr;

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

void hold_axis_position(
        MotorPosController* position_controller,
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

void update_axis_controller(
        MotorPosController* position_controller,
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
    hold_axis_position(
            clamp_out_pos, clamp_out_vel_controller, clamp_out_mode, clamp_vel_out, target_out);
    hold_axis_position(
            clamp_roll_pos, clamp_roll_vel_controller, clamp_roll_mode, clamp_vel_roll, target_roll);
    hold_axis_position(
            clamp_yaw_pos, clamp_yaw_vel_controller, clamp_yaw_mode, clamp_vel_yaw, target_yaw);

    update_axis_controller(
            clamp_out_pos, clamp_out_vel_controller, clamp_out_mode, clamp_vel_out, target_out);
    update_axis_controller(
            clamp_roll_pos,
            clamp_roll_vel_controller,
            clamp_roll_mode,
            clamp_vel_roll,
            target_roll);
    update_axis_controller(
            clamp_yaw_pos, clamp_yaw_vel_controller, clamp_yaw_mode, clamp_vel_yaw, target_yaw);

    clamp_catch_pos->setRef(catch_angle);
    clamp_catch_pos->update();
}

void wait_axis_stall(
        MotorVelController* velocity_controller,
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
    wait_axis_stall(clamp_out_vel_controller, out_reset_lock_threshold, out_reset_hold_ms);
}

void finish_clamp_out_reset()
{
    clamp_vel_out = 0.0f;
    if (motor_clamp_out != nullptr)
    {
        motor_clamp_out->resetAngle();
    }
    clamp_out_vel_controller->getPID().reset();
    target_out     = 0.0f;
    clamp_out_mode = ControlMode::Pos;
}

void reset_clamp_yaw_axis()
{
    clamp_yaw_mode = ControlMode::Vel;
    clamp_vel_yaw  = yaw_reset_seek_velocity;
    wait_axis_stall(clamp_yaw_vel_controller, yaw_reset_lock_threshold, yaw_reset_hold_ms);
}

void finish_clamp_yaw_reset()
{
    clamp_vel_yaw = 0.0f;
    if (motor_clamp_yaw != nullptr)
    {
        motor_clamp_yaw->resetAngle();
    }
    clamp_yaw_vel_controller->getPID().reset();
    target_yaw     = yaw_reset_hold_position;
    clamp_yaw_mode = ControlMode::Pos;
}

} // namespace

void Clamp_Init(void)
{
    clamp_out_vel_controller =
            new MotorVelController(motor_clamp_out, clamp_out_vel_controller_config);
    clamp_yaw_vel_controller =
            new MotorVelController(motor_clamp_yaw, clamp_yaw_vel_controller_config);
    clamp_roll_vel_controller =
            new MotorVelController(motor_clamp_roll, clamp_roll_vel_controller_config);
    clamp_catch_vel_controller =
            new MotorVelController(motor_clamp_catch, clamp_catch_vel_controller_config);

    clamp_out_pos   = new MotorPosController(motor_clamp_out, clamp_out_pos_controller_config);
    clamp_roll_pos  = new MotorPosController(motor_clamp_roll, clamp_roll_pos_controller_config);
    clamp_yaw_pos   = new MotorPosController(motor_clamp_yaw, clamp_yaw_pos_controller_config);
    clamp_catch_pos = new MotorPosController(motor_clamp_catch, clamp_catch_pos_controller_config);

    clamp_out_pos->disable();
    clamp_roll_pos->disable();
    clamp_yaw_pos->disable();
    clamp_out_vel_controller->disable();
    clamp_roll_vel_controller->disable();
    clamp_catch_vel_controller->disable();
    clamp_yaw_vel_controller->enable();
    clamp_catch_pos->enable();

    clamp_out_mode  = ControlMode::Vel;
    clamp_roll_mode = ControlMode::Vel;
    clamp_yaw_mode  = ControlMode::Vel;
}

void Clamp_Control_Init(void)
{
    osThreadNew(Clamp_Control, nullptr, &clamp_attributes);
    const osTimerId_t clamp_timer_handle =
            osTimerNew(Clamp_softTIM, osTimerPeriodic, nullptr, nullptr);
    osTimerStart(clamp_timer_handle, soft_timer_period_ms);
}

void Clamp_Control(void* argument)
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
        osDelay(control_task_delay_ms);
    }
}

void Clamp_softTIM(void* argument)
{
    (void)argument;

    if ((osEventFlagsWait(flags_id, clamp_reset_event_flag, osFlagsWaitAny, 0) &
         (event_wait_mask_base | clamp_reset_event_flag)) == clamp_reset_event_flag)
    {
        control_reset = true;
    }

    if ((osEventFlagsWait(flags_id, clamp_catch_toggle_flag, osFlagsWaitAny, 0) &
         (event_wait_mask_base | clamp_catch_toggle_flag)) == clamp_catch_toggle_flag)
    {
        if (catch_angle == 0.0f)
        {
            catch_angle = catch_closed_angle;
        }
        else
        {
            catch_angle = 0.0f;
        }
    }
}

void APP_Clamp_BeforeUpdate()
{
    Clamp_Init();
    Clamp_Control_Init();
}

void APP_Clamp_Update_1kHz()
{
    clamp_timer_callback();
}

void APP_Clamp_Update_100Hz() {}
