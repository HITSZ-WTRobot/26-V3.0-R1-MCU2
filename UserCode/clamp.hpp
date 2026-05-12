#pragma once

enum class ControlMode
{
    Pos = 0,
    Vel = 1,
};

enum class ResetProcess
{
    Error = 0,
    Start,
    Wait,
    Success,
    Processing,
    Unknown,
};

extern float        clamp_vel_out;
extern float        clamp_vel_yaw;
extern float        clamp_vel_roll;
extern ResetProcess reset_status;

void Clamp_Init(void);
void Clamp_Control(void* argument);
void Clamp_Control_Init(void);
void Clamp_softTIM(void* argument);

void APP_Clamp_BeforeUpdate();
void APP_Clamp_Update_1kHz();
void APP_Clamp_Update_100Hz();
