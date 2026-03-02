#include "armctrl.hpp"

#include "arm_controller.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "gpio.h"
#include "usart.h"

#include <cstdio>
#include <cstring>

namespace
{
constexpr float kControlPeriod = 0.001f;
constexpr float kRatio2        = (3591.0f / (187.0f * 100.0f)) * (16384.0f / (20.0f * 0.3f));
constexpr float kRatio3        = 16384.0f / (20.0f * 0.3f);

Arm::MotorCtrl*  joint1_motor = nullptr;
Arm::MotorCtrl*  joint2_motor = nullptr;
Arm::MotorCtrl*  joint3_motor = nullptr;
Arm::Controller* robot_arm    = nullptr;

constexpr UART_HandleTypeDef* kUartPcHandler = &huart1;
static uint8_t                uart_rx_buf[64];
static volatile bool          uart_rx_flag = false;
static volatile uint16_t      uart_rx_len  = 0;
static volatile bool          is_animating = false;

Arm::Controller::Config BuildArmConfig()
{
    Arm::Controller::Config cfg{};
    cfg.l1  = 0.346f;
    cfg.l2  = 0.382f;
    cfg.l3  = 0.093f;
    cfg.lc1 = 0.171f;
    cfg.lc2 = 0.23769f;
    cfg.lc3 = 0.057f;
    cfg.m1  = 1.2243f;
    cfg.m2  = 0.909f;
    cfg.m3  = 0.6764f;
    cfg.g   = 9.81f;

    cfg.reduction_1 = 1.0f;
    cfg.reduction_2 = 100.0f * 187.0f * 1.5f / 3591.0f;
    cfg.reduction_3 = 1.5f;

    cfg.offset_1 = 0.0f;
    cfg.offset_2 = -164.0f;
    cfg.offset_3 = 90.0f;

    cfg.backlash_1 = 0.0f;
    cfg.backlash_2 = 6.0f;
    cfg.backlash_3 = 3.0f;

    cfg.j1_max_vel  = 50.0f;
    cfg.j1_max_acc  = 50.0f;
    cfg.j1_max_jerk = 500.0f;
    cfg.j2_max_vel  = 20.0f;
    cfg.j2_max_acc  = 120.0f;
    cfg.j2_max_jerk = 500.0f;
    cfg.j3_max_vel  = 3600.0f;
    cfg.j3_max_acc  = 30.0f;
    cfg.j3_max_jerk = 500.0f;

    return cfg;
}
} // namespace

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == kUartPcHandler)
    {
        uart_rx_len  = Size;
        uart_rx_flag = true;
    }
}

void APP_ArmCtrl_BeforeUpdate()
{
    if (robot_arm != nullptr)
        return;
    if (motor_joint1 == nullptr || motor_joint2 == nullptr || motor_element == nullptr)
        return;

    static Arm::MotorCtrl m1(motor_joint1);
    static Arm::MotorCtrl m2(motor_joint2, kRatio2);
    static Arm::MotorCtrl m3(motor_element, kRatio3);

    m1.SetMitParams(150.0f, 3.5f, 0.0f, 2.0f);
    m2.SetMitParams(0.8f, 0.03f, 0.01f, 1.0f);
    m3.SetMitParams(0.6f, 0.03f, 0.01f, 2.0f);

    joint1_motor = &m1;
    joint2_motor = &m2;
    joint3_motor = &m3;

    static Arm::Controller ctrl(*joint1_motor, *joint2_motor, *joint3_motor, BuildArmConfig());
    robot_arm = &ctrl;
}

void APP_ArmCtrl_Init()
{
    if (robot_arm == nullptr)
        return;

    robot_arm->init();
}

void APP_ArmCtrl_Update_1kHz()
{
    if (robot_arm == nullptr)
        return;

    robot_arm->update(kControlPeriod);
}

void APP_ArmCtrl_Update_200Hz() {}

void StatusFeedbackTask(void* argument)
{
    char tx_buf[64];

    osDelay(4000);

    HAL_UARTEx_ReceiveToIdle_DMA(kUartPcHandler, uart_rx_buf, sizeof(uart_rx_buf));

    for (;;)
    {
        if (uart_rx_flag)
        {
            if (uart_rx_len < sizeof(uart_rx_buf))
                uart_rx_buf[uart_rx_len] = 0;
            else
                uart_rx_buf[sizeof(uart_rx_buf) - 1] = 0;

            float cmd_q1        = 0.0f;
            float cmd_q2        = 0.0f;
            float cmd_q3        = 0.0f;
            int   vacuum_state  = 0;
            int   payload_state = 0;
            int   args_count    = 0;

            if ((args_count = std::sscanf(reinterpret_cast<char*>(uart_rx_buf),
                                          "%f, %f, %f, %d, %d",
                                          &cmd_q1,
                                          &cmd_q2,
                                          &cmd_q3,
                                          &vacuum_state,
                                          &payload_state)) >= 4)
            {
                if (robot_arm != nullptr && !is_animating)
                {
                    robot_arm->setJointTarget(cmd_q1, cmd_q2, cmd_q3);

                    if (args_count == 4)
                        payload_state = vacuum_state;

                    if (payload_state)
                        robot_arm->setPayload(0.6f, 0.5f);
                    else
                        robot_arm->setPayload(0.0f, 0.0f);

                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (GPIO_PinState) !vacuum_state);
                }
            }

            uart_rx_flag = false;
            HAL_UARTEx_ReceiveToIdle_DMA(kUartPcHandler, uart_rx_buf, sizeof(uart_rx_buf));
        }

        if (robot_arm != nullptr)
        {
            float cur_q1 = 0.0f;
            float cur_q2 = 0.0f;
            float cur_q3 = 0.0f;
            robot_arm->getJointAngles(cur_q1, cur_q2, cur_q3);

            float pressure_kpa = 0.0f;
            // if (sensor_pressure != nullptr)
            // {
            //     sensor_pressure->update();
            //     pressure_kpa = sensor_pressure->getPressure() / 1000.0f;
            // }

            const int len = std::snprintf(tx_buf,
                                          sizeof(tx_buf),
                                          "%.2f, %.2f, %.2f, %.2f, %d\n",
                                          cur_q1,
                                          cur_q2,
                                          cur_q3,
                                          pressure_kpa,
                                          is_animating ? 1 : 0);

            HAL_UART_Transmit_DMA(kUartPcHandler,
                                  reinterpret_cast<uint8_t*>(tx_buf),
                                  static_cast<uint16_t>(len));
        }

        osDelay(50);
    }
}

void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);

    for (;;)
    {
        osDelay(1000);
    }
}
