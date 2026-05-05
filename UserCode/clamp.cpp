#include "clamp.hpp"
#include "main.h"

#include <cmath>

using namespace Device::motor;

#define AprilTag2MACHINE_X 0
#define AprilTag2MACHINE_Y 0

Motor_PosCtrl_t* clamp_out_pos   = nullptr;
Motor_PosCtrl_t* clamp_yaw_pos   = nullptr;
Motor_PosCtrl_t* clamp_roll_pos  = nullptr;
Motor_PosCtrl_t* clamp_catch_pos = nullptr;

Motor_VelCtrl_t* clamp_out_vel   = nullptr;
Motor_VelCtrl_t* clamp_yaw_vel   = nullptr;
Motor_VelCtrl_t* clamp_roll_vel  = nullptr;
Motor_VelCtrl_t* clamp_catch_vel = nullptr;

enum Control_Mode clamp_out_mode;  // 矛杆推出模式
enum Control_Mode clamp_roll_mode; // 矛杆翻转模式

static uint32_t time_now;
static uint32_t time_lockedstart;
bool            control_reset = 0;

float target_out  = 0;
float target_yaw  = 0;
float target_roll = 0;

float clamp_vel_out  = 0;
float clamp_vel_yaw  = 0;
float clamp_vel_roll = 0;

float catch_angle       = 0;
bool  catch_status_last = 0; // 上一个周期的抓取状态
bool  catch_status      = 0; //(1为准备触发，0为已经触发)

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
    controllers::MotorVelController::Config clamp_out_vel_cfg{};
    clamp_out_vel_cfg.pid.Kp             = 500.0f;
    clamp_out_vel_cfg.pid.Ki             = 0.1f;
    clamp_out_vel_cfg.pid.Kd             = 0.0f;
    clamp_out_vel_cfg.pid.abs_output_max = 8000.0f;

    controllers::MotorVelController::Config clamp_yaw_vel_cfg{};
    clamp_yaw_vel_cfg.pid.Kp             = 45.0f;
    clamp_yaw_vel_cfg.pid.Ki             = 0.15f;
    clamp_yaw_vel_cfg.pid.Kd             = 0.0f;
    clamp_yaw_vel_cfg.pid.abs_output_max = 8000.0f;

    controllers::MotorVelController::Config clamp_roll_vel_cfg{};
    clamp_roll_vel_cfg.pid.Kp             = 500.0f;
    clamp_roll_vel_cfg.pid.Ki             = 0.1f;
    clamp_roll_vel_cfg.pid.Kd             = 0.0f;
    clamp_roll_vel_cfg.pid.abs_output_max = 8000.0f;

    controllers::MotorVelController::Config clamp_catch_vel_cfg{};
    clamp_catch_vel_cfg.pid.Kp             = 500.0f;
    clamp_catch_vel_cfg.pid.Ki             = 0.1f;
    clamp_catch_vel_cfg.pid.Kd             = 0.0f;
    clamp_catch_vel_cfg.pid.abs_output_max = 4500.0f;

    controllers::MotorPosController::Config clamp_out_pos_cfg{};
    clamp_out_pos_cfg.velocity_pid.Kp             = 500.0f;
    clamp_out_pos_cfg.velocity_pid.Ki             = 0.10f;
    clamp_out_pos_cfg.velocity_pid.Kd             = 0.00f;
    clamp_out_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
    clamp_out_pos_cfg.position_pid.Kp             = 2.0f;
    clamp_out_pos_cfg.position_pid.Ki             = 0.0f;
    clamp_out_pos_cfg.position_pid.Kd             = 0.2f;
    clamp_out_pos_cfg.position_pid.abs_output_max = 400.0f;
    clamp_out_pos_cfg.pos_vel_freq_ratio          = 10;

    controllers::MotorPosController::Config clamp_roll_pos_cfg{};
    clamp_roll_pos_cfg.velocity_pid.Kp             = 500.0f;
    clamp_roll_pos_cfg.velocity_pid.Ki             = 0.10f;
    clamp_roll_pos_cfg.velocity_pid.Kd             = 0.0f;
    clamp_roll_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
    clamp_roll_pos_cfg.position_pid.Kp             = 2.0f;
    clamp_roll_pos_cfg.position_pid.Ki             = 0.0f;
    clamp_roll_pos_cfg.position_pid.Kd             = 0.2f;
    clamp_roll_pos_cfg.position_pid.abs_output_max = 400.0f;
    clamp_roll_pos_cfg.pos_vel_freq_ratio          = 10;

    controllers::MotorPosController::Config clamp_yaw_pos_cfg{};
    clamp_yaw_pos_cfg.velocity_pid.Kp             = 12.0f;
    clamp_yaw_pos_cfg.velocity_pid.Ki             = 0.20f;
    clamp_yaw_pos_cfg.velocity_pid.Kd             = 5.00f;
    clamp_yaw_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
    clamp_yaw_pos_cfg.position_pid.Kp             = 80.0f;
    clamp_yaw_pos_cfg.position_pid.Ki             = 1.00f;
    clamp_yaw_pos_cfg.position_pid.Kd             = 0.00f;
    clamp_yaw_pos_cfg.position_pid.abs_output_max = 2000.0f;
    clamp_yaw_pos_cfg.pos_vel_freq_ratio          = 10;

    controllers::MotorPosController::Config clamp_catch_pos_cfg{};
    clamp_catch_pos_cfg.velocity_pid.Kp             = 500.0f;
    clamp_catch_pos_cfg.velocity_pid.Ki             = 0.10f;
    clamp_catch_pos_cfg.velocity_pid.Kd             = 0.0f;
    clamp_catch_pos_cfg.velocity_pid.abs_output_max = 4500.0f;
    clamp_catch_pos_cfg.position_pid.Kp             = 2.0f;
    clamp_catch_pos_cfg.position_pid.Ki             = 0.0f;
    clamp_catch_pos_cfg.position_pid.Kd             = 0.2f;
    clamp_catch_pos_cfg.position_pid.abs_output_max = 400.0f;
    clamp_catch_pos_cfg.pos_vel_freq_ratio          = 10;

    clamp_out_vel   = new Motor_VelCtrl_t(motor_clamp_out, clamp_out_vel_cfg);
    clamp_yaw_vel   = new Motor_VelCtrl_t(motor_clamp_yaw, clamp_yaw_vel_cfg);
    clamp_roll_vel  = new Motor_VelCtrl_t(motor_clamp_roll, clamp_roll_vel_cfg);
    clamp_catch_vel = new Motor_VelCtrl_t(motor_clamp_catch, clamp_catch_vel_cfg);
    clamp_out_pos   = new Motor_PosCtrl_t(motor_clamp_out, clamp_out_pos_cfg);
    clamp_roll_pos  = new Motor_PosCtrl_t(motor_clamp_roll, clamp_roll_pos_cfg);
    clamp_yaw_pos   = new Motor_PosCtrl_t(motor_clamp_yaw, clamp_yaw_pos_cfg);
    clamp_catch_pos = new Motor_PosCtrl_t(motor_clamp_catch, clamp_catch_pos_cfg);

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
    osTimerStart(drawer_timHandle, 20); // 20 ms periodic timer
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
    clamp_vel_out  = 100;
    reset_status   = start;
    while (std::fabs(clamp_out_vel->getPID().getOutput()) < 1500.0f)
    {
        osDelay(1);
    }

    while (reset_status != success)
    {
        reset_status = processing;
        while (std::fabs(clamp_out_vel->getPID().getOutput()) < 1500.0f)
        {
            osDelay(1);
        }
        uint32_t time_start = HAL_GetTick();
        while (std::fabs(clamp_out_vel->getPID().getOutput()) >= 1500.0f)
        {
            reset_status = wait;
            if (HAL_GetTick() - time_start >= 500)
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
    for (;;)
    {
        if (control_reset == 1)
        {
            reset();
            clamp_vel_out = 0;
            if (motor_clamp_out != nullptr)
            {
                motor_clamp_out->resetAngle();
            }
            clamp_out_vel->getPID().reset();
            control_reset = false;
        }
        osDelay(100);
    }
}

/**
 *
 * @param arguement
 */
void Clamp_softTIM(void* arguement)
{
    (void)arguement;

    if ((osEventFlagsWait(flags_id, 0x00000080U, osFlagsWaitAny, 0) & 0xFF000080U) == 0x00000080U)
    {
        control_reset = 1;
    }
    if ((osEventFlagsWait(flags_id, 0x00000040U, osFlagsWaitAny, 0) & 0xFF000040U) == 0x00000040U)
    {
        if (catch_angle == 0)
        {
            catch_angle = 150;
        }
        else
        {
            catch_angle = 0;
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
