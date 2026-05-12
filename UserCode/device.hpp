/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-02-02
 */
#pragma once

#include "DM.hpp"
#include "I2CBusDMA.hpp"
#include "I2CUpdateManager.hpp"
#include "XGZP6847DDevice.hpp"
#include "dji.hpp"
#include "can.h"
#include "i2c.h"
#include "usart.h"

// Joint 1: DM-J10010L (CAN2)
extern motors::DMMotor *motor_joint1;

// Joint 2: DJI M3508 (CAN1)
extern motors::DJIMotor *motor_joint2;

// Joint 3 (Gripper): DM J4310 (CAN2)
extern motors::DMMotor *motor_element; // Element/Gripper

// I2C 总线与更新管理器
extern I2CBusDMA *i2c_bus_1;
extern I2CUpdateManager *i2c_manager_1;

// Pressure Sensor
extern XGZP6847DDevice *sensor_pressure;

void APP_Device_Init();
void APP_Device_Update_1kHz();
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();
float APP_Device_GetPressureKpa();
