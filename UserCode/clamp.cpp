#include "clamp.hpp"
#include "main.h"

#include <cmath>

using namespace Device::motor;
namespace ProjectClampConfig = AppConfig::Clamp;

Motor_PosCtrl_t* clamp_out_pos   = nullptr;
Motor_PosCtrl_t* clamp_yaw_pos   = nullptr;
Motor_PosCtrl_t* clamp_roll_pos  = nullptr;
Motor_PosCtrl_t* clamp_catch_pos = nullptr;

Motor_VelCtrl_t* clamp_out_vel   = nullptr;
Motor_VelCtrl_t* clamp_yaw_vel   = nullptr;
Motor_VelCtrl_t* clamp_roll_vel  = nullptr;
Motor_VelCtrl_t* clamp_catch_vel = nullptr;

enum Control_Mode clamp_out_mode;
enum Control_Mode clamp_roll_mode;

bool control_reset = false;

float target_out  = 0.0f;
float target_yaw  = 0.0f;
float target_roll = 0.0f;

float clamp_vel_out  = 0.0f;
float clamp_vel_yaw  = 0.0f;
float clamp_vel_roll = 0.0f;

float catch_angle = 0.0f;

osTimerId_t drawer_timHandle;

enum PROCESS reset_status = unknown;

osThreadId_t         clampHandle      = nullptr;
const osThreadAttr_t clamp_attributes = {
    .name       = "clamp",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal1,
};

void Clamp_Init(void)
{
    clamp_out_vel   = new Motor_VelCtrl_t(motor_clamp_out, ProjectClampConfig::OutVelControllerCfg);
    clamp_yaw_vel   = new Motor_VelCtrl_t(motor_clamp_yaw, ProjectClampConfig::YawVelControllerCfg);
    clamp_roll_vel  = new Motor_VelCtrl_t(motor_clamp_roll, ProjectClampConfig::RollVelControllerCfg);
    clamp_catch_vel = new Motor_VelCtrl_t(
            motor_clamp_catch, ProjectClampConfig::CatchVelControllerCfg);

    clamp_out_pos =
            new Motor_PosCtrl_t(motor_clamp_out, ProjectClampConfig::OutPosControllerCfg);
    clamp_roll_pos =
            new Motor_PosCtrl_t(motor_clamp_roll, ProjectClampConfig::RollPosControllerCfg);
    clamp_yaw_pos =
            new Motor_PosCtrl_t(motor_clamp_yaw, ProjectClampConfig::YawPosControllerCfg);
    clamp_catch_pos =
            new Motor_PosCtrl_t(motor_clamp_catch, ProjectClampConfig::CatchPosControllerCfg);

    clamp_out_pos->disable();
    clamp_roll_pos->disable();
    clamp_yaw_pos->disable();
    clamp_out_vel->disable();
    clamp_roll_vel->disable();
    clamp_roll_pos->disable();
    clamp_yaw_vel->enable();
    clamp_catch_pos->enable();
    clamp_out_mode  = VEL_Control;
    clamp_roll_mode = VEL_Control;
}

void Clamp_Control_Init(void)
{
    clampHandle      = osThreadNew(Clamp_Control, NULL, &clamp_attributes);
    drawer_timHandle = osTimerNew(Clamp_softTIM, osTimerPeriodic, NULL, NULL);
    osTimerStart(drawer_timHandle, ProjectClampConfig::SoftTimerPeriodMs);
}

static void clamp_TIM_callback(void)
{
    switch (clamp_out_mode)
    {
    case POS_Control:
        clamp_out_vel->disable();
        clamp_out_pos->enable();
        clamp_out_pos->setRef(target_out);
        clamp_out_pos->update();
        break;

    case VEL_Control:
        clamp_out_pos->disable();
        clamp_out_vel->enable();
        clamp_out_vel->setRef(clamp_vel_out);
        clamp_out_vel->update();
        break;

    default:
        break;
    }

    switch (clamp_roll_mode)
    {
    case POS_Control:
        clamp_roll_vel->disable();
        clamp_roll_pos->enable();
        clamp_roll_pos->setRef(target_roll);
        clamp_roll_pos->update();
        break;

    case VEL_Control:
        clamp_roll_pos->disable();
        clamp_roll_vel->enable();
        clamp_roll_vel->setRef(clamp_vel_roll);
        clamp_roll_vel->update();
        break;

    default:
        break;
    }

    clamp_catch_pos->setRef(catch_angle);
    clamp_catch_pos->update();
    clamp_yaw_vel->setRef(clamp_vel_yaw);
    clamp_yaw_vel->update();
}

static void reset()
{
    clamp_out_mode = VEL_Control;
    clamp_vel_out  = ProjectClampConfig::ResetSeekVelocity;
    reset_status   = start;

    while (std::fabs(clamp_out_vel->getPID().getOutput()) < ProjectClampConfig::ResetLockThreshold)
    {
        osDelay(1);
    }

    while (reset_status != success)
    {
        reset_status = processing;
        while (std::fabs(clamp_out_vel->getPID().getOutput()) <
               ProjectClampConfig::ResetLockThreshold)
        {
            osDelay(1);
        }

        const uint32_t time_start = HAL_GetTick();
        while (std::fabs(clamp_out_vel->getPID().getOutput()) >=
               ProjectClampConfig::ResetLockThreshold)
        {
            reset_status = wait;
            if (HAL_GetTick() - time_start >= ProjectClampConfig::ResetHoldMs)
            {
                reset_status = success;
                break;
            }
            osDelay(1);
        }
    }
}

void Clamp_Control(void* argument)
{
    (void)argument;

    for (;;)
    {
        if (control_reset)
        {
            reset();
            clamp_vel_out = 0.0f;
            if (motor_clamp_out != nullptr)
            {
                motor_clamp_out->resetAngle();
            }
            clamp_out_vel->getPID().reset();
            control_reset = false;
        }
        osDelay(ProjectClampConfig::ControlTaskDelayMs);
    }
}

void Clamp_softTIM(void* arguement)
{
    (void)arguement;

    if ((osEventFlagsWait(flags_id, 0x00000080U, osFlagsWaitAny, 0) & 0xFF000080U) == 0x00000080U)
    {
        control_reset = true;
    }
    if ((osEventFlagsWait(flags_id, 0x00000040U, osFlagsWaitAny, 0) & 0xFF000040U) == 0x00000040U)
    {
        if (catch_angle == 0.0f)
        {
            catch_angle = ProjectClampConfig::CatchClosedAngle;
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
    clamp_TIM_callback();
}

void APP_Clamp_Update_100Hz() {}
