/**
 * @file    system.hpp
 * @author  syhanjin
 * @date    2025-12-02
 * @brief   system defines
 */
#pragma once

#include "cmsis_os2.h"

/**
 * 系统事件 Event
 */
extern osEventFlagsId_t systemEventHandle;

#define SYSTEM_INITIALIZED (0x00000001U)

/**
 * 等待系统初始化
 */
static uint32_t System_WaitInitialize()
{
    return osEventFlagsWait(systemEventHandle,
                            SYSTEM_INITIALIZED,
                            osFlagsWaitAny | osFlagsNoClear,
                            osWaitForever);
}
