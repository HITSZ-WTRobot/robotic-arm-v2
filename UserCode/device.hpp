/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-02-02
 */
#pragma once

#include "DM.hpp"
#include "dji.hpp"
// #include "motors/Unitree/Unitree.hpp"
// #include "XGZP6847D.hpp"
#include "can.h"
#include "i2c.h"
#include "usart.h"

// Joint 1: DM-J10010L (CAN2)
extern motors::DMMotor *motor_joint1;

// Joint 2: DJI M3508 (CAN1)
extern motors::DJIMotor *motor_joint2;

// Joint 3 (Gripper): DJI M3508 (CAN1)
extern motors::DJIMotor *motor_element; // Element/Gripper

// Pressure Sensor
// extern sensors::pressure::XGZP6847D *sensor_pressure;

void APP_Device_Init();
void APP_Device_Update_1kHz();
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();
