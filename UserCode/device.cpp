#include "device.hpp"

#include "can.h"
#include "cmsis_os2.h"

namespace Device
{
constexpr motors::DJIMotor::Config rotate_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M3508_C620,
    .id1       = 1,
    .auto_zero = false,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config catch_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 2,
    .auto_zero = false,
    .reverse   =  false,
};

constexpr motors::DJIMotor::Config raiseandlower_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M3508_C620,
    .id1       = 3,
    .auto_zero = false,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_out_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 1,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_roll_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 2,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_yaw_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M3508_C620,
    .id1       = 3,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_catch_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 4,
    .auto_zero = true,
    .reverse   = false,
};

void can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);

    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    CAN_InitMainCallback(&hcan1);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_InitMainCallback(&hcan2);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void motor_init()
{
    using motors::DJIMotor;

    motor::rotate_motor        = new DJIMotor(rotate_motor_config);
    motor::catch_motor         = new DJIMotor(catch_motor_config);
    motor::raiseandlower_motor = new DJIMotor(raiseandlower_motor_config);
    motor::motor_clamp_out     = new DJIMotor(motor_clamp_out_config);
    motor::motor_clamp_roll    = new DJIMotor(motor_clamp_roll_config);
    motor::motor_clamp_yaw     = new DJIMotor(motor_clamp_yaw_config);
    motor::motor_clamp_catch   = new DJIMotor(motor_clamp_catch_config);
}

void app_device_init()
{
    can_init();
    motor_init();
}

bool app_device_IsAllConnected()
{
    bool all_connected = true;
    all_connected &= (motor::rotate_motor != nullptr) && motor::rotate_motor->isConnected();
    all_connected &=
            (motor::raiseandlower_motor != nullptr) && motor::raiseandlower_motor->isConnected();
    all_connected &= (motor::catch_motor != nullptr) && motor::catch_motor->isConnected();
    all_connected &= (motor::motor_clamp_out != nullptr) && motor::motor_clamp_out->isConnected();
    all_connected &= (motor::motor_clamp_roll != nullptr) &&
                     motor::motor_clamp_roll->isConnected();
    all_connected &= (motor::motor_clamp_yaw != nullptr) && motor::motor_clamp_yaw->isConnected();
    all_connected &= (motor::motor_clamp_catch != nullptr) &&
                     motor::motor_clamp_catch->isConnected();
    return all_connected;
}

void app_device_WaitConnections()
{
    while (!app_device_IsAllConnected())
    {
        osDelay(1);
    }
}

void update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

} // namespace Device
