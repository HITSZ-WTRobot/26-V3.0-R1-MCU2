#pragma once

#include "cmsis_os2.h"

#define KEY1 0x00000001U
#define KEY2 0x00000002U
#define KEY3 0x00000004U
#define KEY4 0x00000008U
#define KEY5 0x00000010U
#define KEY6 0x00000020U
#define KEY7 0x00000040U
#define KEY8 0x00000080U
#define KEY9 0x00000100U
#define KEY10 0x00000200U
#define KEY11 0x00000400U
#define KEY12 0x00000800U
#define ERROR_MASK 0xFF000000U
#define DIP4_MASK 0x00180000U
#define DIP4_CASE0 0x00000000U
#define DIP4_CASE1 0x00080000U
#define DIP4_CASE2 0x00100000U

extern osEventFlagsId_t flags_id;
extern uint32_t button_state ;
void                    flags_create();