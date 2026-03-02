/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-02-02
 */

#include "device.hpp"
#include "DM.hpp"
#include "dma.h"

// Define global pointers
motors::DMMotor*  motor_joint1  = nullptr;
motors::DJIMotor* motor_joint2  = nullptr;
motors::DJIMotor* motor_element = nullptr;
// sensors::pressure::XGZP6847D *sensor_pressure = nullptr;

void APP_Device_Init()
{
    // 1. Initialize CAN Filters
    // CAN1 for DJI
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    // CAN2 for DM (Filter bank 14)
    // hcan, filter_bank, master_id
    motors::DMMotor::CAN_FilterInit(&hcan2, 14,
                                    0x114); // 0x114 is default master ID

    // Register Callbacks
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan2, motors::DMMotor::CANBaseReceiveCallback);

    // Start CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 2. Create Motor Objects

    // Joint 1: DM-J10010L
    motors::DMMotor::Config dm_config = { .hcan        = &hcan2,
                                          .id0         = 1, // ID 1 (0-based index)
                                          .type        = motors::DMMotor::Type::J10010L_2EC,
                                          .mode        = motors::DMMotor::Mode::MIT,
                                          .pos_max_rad = 12.5f,  // Example max position in radians
                                          .vel_max_rad = 25.0f,  // Example max velocity in rad/s
                                          .tor_max     = 200.0f, // Example max torque in Nm
                                          .auto_zero   = true,
                                          .reverse     = true,
                                          .reduction_rate = 1.0f };
    motor_joint1                      = new motors::DMMotor(dm_config);

    // Joint 2: DJI M3508
    motor_joint2 = new motors::DJIMotor({ .hcan      = &hcan1,
                                          .type      = motors::DJIMotor::Type::M3508_C620,
                                          .id1       = 1,
                                          .auto_zero = true,
                                          .reverse   = false });

    // Joint 3 (Gripper): DJI M3508
    motor_element = new motors::DJIMotor({ .hcan      = &hcan1,
                                           .type      = motors::DJIMotor::Type::M3508_C620,
                                           .id1       = 2,
                                           .auto_zero = true,
                                           .reverse   = true });

    // 3. Sensors
    // XGZP6847D: I2C1, Range 200.0f
    // sensor_pressure = new sensors::pressure::XGZP6847D({ .hi2c = &hi2c1, .k = 32.0f, .b = 0.0f });
}

void APP_Device_Update_1kHz()
{
    // Send commands for DJI motors (DM sends immediately on setMitCommand)
    // Send both group 1 (1-4) and possibly group 2 (5-8) if needed,
    // but here we use ID 1 and 2, so Group 1 is enough.
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool APP_Device_isAllConnected()
{
    bool m1_ok = motor_joint1 && motor_joint1->isConnected();
    bool m2_ok = motor_joint2 && motor_joint2->isConnected();
    bool m3_ok = motor_element && motor_element->isConnected();

    return m1_ok && m2_ok && m3_ok;
}

void APP_Device_WaitConnections()
{
    uint32_t tick = HAL_GetTick();
    while (!APP_Device_isAllConnected())
    {
        if (HAL_GetTick() - tick > 10000)
        {
            // Timeout
            break;
        }
        HAL_Delay(10);
    }
}
