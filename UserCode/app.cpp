/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-02-01
 */
#include "app.hpp"

#include "arm_controller.hpp"
#include "armctrl.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "dji.hpp"
#include "system.hpp"
#include "tim.h"

size_t prescaler = 0;

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    APP_ArmCtrl_Update_1kHz();

    APP_Device_Update_1kHz();

    service::Watchdog::EatAll();
}

extern "C" void TIM_Callback_200Hz(TIM_HandleTypeDef* htim)
{
    APP_ArmCtrl_Update_200Hz();
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    APP_Device_Init();

    APP_ArmCtrl_BeforeUpdate();

    // HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_200Hz);
    // HAL_TIM_Base_Start_IT(&htim13);

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    motor_joint1->ping(); // 达妙电机的心跳包
    motor_element->ping();
    osDelay(10);
    motor_joint1->ping(); // 达妙电机的心跳包
    motor_element->ping();
    osDelay(10);
    // 等待各设备连接
    APP_Device_WaitConnections();

    /**
     * 等待各种东西更新
     */

    osDelay(1000);
    APP_ArmCtrl_Init();
    osDelay(1000);
    // 启动定时器
    // HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    // HAL_TIM_Base_Start_IT(&htim6);

    osEventFlagsSet(systemEventHandle, SYSTEM_INITIALIZED);

    /* 初始化完成后退出线程 */
    osThreadExit();
}