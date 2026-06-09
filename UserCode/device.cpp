/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-02-02
 */

#include "device.hpp"
#include "DM.hpp"
#include "cmsis_os2.h"
#include "dma.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"

// Define global pointers
motors::DMMotor*  motor_joint1    = nullptr;
motors::DJIMotor* motor_joint2    = nullptr;
motors::DMMotor*  motor_element   = nullptr;
I2CBusDMA*        i2c_bus_1       = nullptr;
I2CUpdateManager* i2c_manager_1   = nullptr;
XGZP6847DDevice*  sensor_pressure = nullptr;

void APP_Device_Init()
{
    // 1. Initialize CAN Filters
    // CAN1 for DJI
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    // CAN2 for DM (Filter bank 14)
    // hcan, filter_bank, master_id
    motors::DMMotor::CAN_FilterInit(&hcan2, 14,
                                    0x114); // 0x114 is master ID

    // Register Callbacks
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan2, motors::DMMotor::CANBaseReceiveCallback);

    // Start CAN
    HAL_CAN_RegisterCallback(&hcan1,
                             HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                             DJI_CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_RegisterCallback(&hcan2,
                             HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                             DM_CAN_Fifo0ReceiveCallback);
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
                                          .auto_zero   = false,
                                          .reverse     = false,
                                          .reduction_rate = 1.0f };
    motor_joint1                      = new motors::DMMotor(dm_config);
    motor_joint1->disable();
    motor_joint1->ping();

    // Joint 2: DJI M3508
    motor_joint2 = new motors::DJIMotor({ .hcan      = &hcan1,
                                          .type      = motors::DJIMotor::Type::M3508_C620,
                                          .id1       = 1,
                                          .auto_zero = true,
                                          .reverse   = false });

    // Joint 3 (Gripper): DM J4310
    motors::DMMotor::Config dm_config_2 = { .hcan        = &hcan2,
                                            .id0         = 2, // ID 2 (0-based index)
                                            .type        = motors::DMMotor::Type::J4310_2EC,
                                            .mode        = motors::DMMotor::Mode::MIT,
                                            .pos_max_rad = 6.3f,  // Example max position in radians
                                            .vel_max_rad = 25.0f, // Example max velocity in rad/s
                                            .tor_max     = 12.0f, // Example max torque in Nm
                                            .auto_zero   = false,
                                            .reverse     = false,
                                            .reduction_rate = 1.0f };
    motor_element                       = new motors::DMMotor(dm_config_2);

    // 3. Sensors
    // XGZP6847D: I2C1, 量程 200.0 kPa
    static I2CBusDMA i2c1_bus(&hi2c1, { GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9, GPIO_AF4_I2C1 });
    static I2CUpdateManager i2c_manager(i2c1_bus);
    static XGZP6847DDevice  pressure_sensor(300.0f);

    i2c_bus_1       = &i2c1_bus;
    i2c_manager_1   = &i2c_manager;
    sensor_pressure = &pressure_sensor;

    (void) i2c_manager_1->registerDevice(*sensor_pressure, 50U, 0U, 20U);

    I2CUpdateManager::Config manager_config{};
    manager_config.task_name        = "I2C1Mgr";
    manager_config.stack_size_bytes = 512U * sizeof(uint32_t);
    manager_config.priority         = osPriorityAboveNormal;
    manager_config.max_sleep_ms     = 1000U;
    (void) i2c_manager_1->start(manager_config);
}

void APP_Device_Update_1kHz()
{
    // Send commands for DJI motors (DM sends immediately on setMitCommand)
    // Send both group 1 (1-4) and possibly group 2 (5-8) if needed,
    // but here we use ID 1 and 2, so Group 1 is enough.
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}
bool m1_ok, m2_ok, m3_ok;
bool APP_Device_isAllConnected()
{
    m1_ok = motor_joint1 && motor_joint1->isConnected();
    m2_ok = motor_joint2 && motor_joint2->isConnected();
    m3_ok = motor_element && motor_element->isConnected();

    return m1_ok && m2_ok && m3_ok;
    // return true;
}

void APP_Device_WaitConnections()
{
    uint32_t tick = HAL_GetTick();
    while (!APP_Device_isAllConnected())
    {
        motor_joint1->ping(); // 达妙电机的心跳包
        motor_element->ping();
        if (HAL_GetTick() - tick > 10000)
        {
            // Timeout
            Error_Handler();
            break;
        }
        HAL_Delay(5);
    }
}

float APP_Device_GetPressureKpa()
{
    if (sensor_pressure == nullptr)
        return 0.0f;

    if (!sensor_pressure->hasValidData())
        return 0.0f;

    constexpr uint32_t kPressureStaleMs = 100U;
    const auto         sample           = sensor_pressure->snapshot();

    if (!sample.valid)
        return 0.0f;

    if (!sensor_pressure->isDataFresh(HAL_GetTick(), kPressureStaleMs))
        return 0.0f;

    return sample.pressure_pa / 1000.0f;
}
