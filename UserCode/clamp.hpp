#pragma once

#include "cmsis_os2.h"
#include "device.hpp"
#include "flags.hpp"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"

#define MAX_VEL_ROLL 100.0f
#define MAX_VEL_YAW  10.0f

enum Control_Mode
{
    POS_Control = 0, // 位置控制
    VEL_Control = 1  // 速度控制
};

enum PROCESS
{
    error = 0,
    start,
    wait,
    success,
    processing,
    unknown,
};

using Motor_PosCtrl_t = controllers::MotorPosController;
using Motor_VelCtrl_t = controllers::MotorVelController;

extern Motor_PosCtrl_t* clamp_out_pos;
extern Motor_PosCtrl_t* clamp_yaw_pos;
extern Motor_PosCtrl_t* clamp_roll_pos;
extern Motor_PosCtrl_t* clamp_catch_pos;

extern Motor_VelCtrl_t* clamp_out_vel;
extern Motor_VelCtrl_t* clamp_yaw_vel;
extern Motor_VelCtrl_t* clamp_roll_vel;
extern Motor_VelCtrl_t* clamp_catch_vel;

extern float        clamp_vel_out;
extern float        clamp_vel_yaw;
extern float        clamp_vel_roll;
extern bool         control_reset;
extern enum PROCESS reset_status;

void Clamp_Init(void);
void Clamp_Control(void* argument);
void Clamp_Control_Init(void);
void Clamp_softTIM(void* arguement);

void APP_Clamp_BeforeUpdate();
void APP_Clamp_Update_1kHz();
void APP_Clamp_Update_100Hz();
