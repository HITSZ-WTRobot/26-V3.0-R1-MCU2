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
void ClampControl(void* argument);
void clamp_control_init(void);
void clamp_softTIM(void* argument);

void app_clamp_init();
void app_clamp_update_1kHz();
