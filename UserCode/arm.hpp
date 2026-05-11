#pragma once

#define ARM_MAX_VEL_OUT    50.0f 
#define ARM_MAX_VEL_ROTATE 50.0f 
#define ARM_MAX_VEL_HEIGHT 50.0f 

void Arm_TIM_Callback(void);
void Arm_Init(void);

enum ArmAutoCatchLevel {
	ARM_AUTO_CATCH_LOW = 0,
	ARM_AUTO_CATCH_MID = 1,
	ARM_AUTO_CATCH_HIGH = 2,
};

extern float arm_vel_out;
extern float arm_vel_rotate;
extern float arm_vel_height;
extern float arm_vel_out_last;
extern float arm_vel_rotate_last;
extern float arm_vel_height_last;
extern float arm_pos_out;
extern float arm_pos_rotate;
extern float arm_pos_height;



bool Arm_AutoCatchStart(ArmAutoCatchLevel level);
bool Arm_AutoCatchBusy();
void Arm_AutoCatchAbortKeepPump();
void Arm_SetAutoRetreatLength(float length_m);

void APP_Arm_BeforeUpdate();
void APP_Arm_Update_1kHz();
void APP_Arm_Update_100Hz();
void Arm_Rotate_Out(bool enable);
void Arm_Rotate_Back(bool enable);