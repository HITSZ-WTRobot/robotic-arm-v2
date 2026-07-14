/**
 * @file    armctrl.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

void APP_ArmCtrl_BeforeUpdate();
void APP_ArmCtrl_Init();
void APP_ArmCtrl_Update_1kHz();
void APP_ArmCtrl_Update_200Hz();
void StatusFeedbackTask(void* argument);
void MotorCtrl(void* argument);
#ifdef __cplusplus
}
#endif