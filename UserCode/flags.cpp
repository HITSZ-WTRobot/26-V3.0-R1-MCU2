#include "flags.hpp"

// 信号量低24位有效，其中低9位表示按键有没有被按下过
osEventFlagsId_t flags_id;
uint32_t button_state;
void             flags_create(void)
{
    button_state = 0U;
    flags_id = osEventFlagsNew(NULL);
}