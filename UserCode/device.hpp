#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "dji.hpp"

namespace Device
{

namespace motor
{
inline motors::DJIMotor* rotate_motor;
inline motors::DJIMotor* raiseandlower_motor;
inline motors::DJIMotor* catch_motor;
inline motors::DJIMotor* motor_clamp_out;
inline motors::DJIMotor* motor_clamp_roll;
inline motors::DJIMotor* motor_clamp_catch;
inline motors::DJIMotor* motor_clamp_yaw;
} // namespace motor 
void update_1kHz(); // 设备状态更新函数，用于在中断中调用
void app_device_init();

} // namespace Device
#endif // _DEVICE_H_
