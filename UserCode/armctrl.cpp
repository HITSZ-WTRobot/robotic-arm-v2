#include "armctrl.hpp"

#include "arm_controller.hpp"
#include "arm_motor_mit.hpp"
#include "arm_slave.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "gpio.h"
#include "usart.h"
#include "UartRxSync.hpp"

#include <cstdio>
#include <cstring>
#include <cstdint>

namespace
{
constexpr float kRatio2 = (3591.0f / (187.0f * 100.0f)) * (16384.0f / (20.0f * 0.3f));
// constexpr float kRatio3 = 16384.0f / (20.0f * 0.3f);

Arm::MITMotorCtrl* joint1_motor = nullptr;
Arm::MITMotorCtrl* joint2_motor = nullptr;
Arm::MITMotorCtrl* joint3_motor = nullptr;
Arm::Controller*   robot_arm    = nullptr;
Arm::Slave*        arm_slave    = nullptr;

constexpr UART_HandleTypeDef* kUartPcHandler = &huart1;

constexpr uint16_t kSof          = 0xAA55;
constexpr uint8_t  kFlagVacuum   = 0x80;
constexpr uint8_t  kTrajFrameLen = 2 + 2 + 24 + 1;

static bool    traj_base_valid   = false;
static uint8_t last_vacuum_state = 0;

static_assert(sizeof(Arm::Slave::TrajPoint) >= (kTrajFrameLen - 2),
              "TrajPoint is smaller than payload");

struct __attribute__((packed)) FeedbackFrame // 返回帧定义
{
    uint16_t sof;
    float    q1;
    float    q2;
    float    q3;
    float    dq1;
    float    dq2;
    float    dq3;
    float    pressure_kpa;
};

/**
 * PC Trajectory Receiver:
 * - 接收来自上位机的轨迹点，解析后通过 `arm_slave` 下发给控制器。
 * - 还负责控制末端吸盘的开关（通过 GPIOA PIN6）。
 *
 * 帧结构（共 29 字节）：
 * - SOF (2 bytes): 0x55AA
 * - q1, q2, q3 (4 bytes each): 目标关节角度，单位为度
 * - dq1, dq2, dq3 (4 bytes each): 目标关节角速度，单位为度/s
 * - flags (1 byte): 位0-吸盘状态（1=吸住），位6-负载状态（1=有负载）
 *
 * 注意：这个模块直接操作全局指针 `arm_slave` 和 `robot_arm`，
 * 因此必须确保它们在使用前已经正确初始化。
 */
class PcTrajRx final : public protocol::UartRxSync<2, kTrajFrameLen>
{
public:
    explicit PcTrajRx(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

protected:
    const std::array<uint8_t, 2>& header() const override
    {
        static constexpr std::array<uint8_t, 2> kHeader = { 0x55, 0xAA };
        return kHeader;
    }

    bool decode(const uint8_t data[kTrajFrameLen - 2]) override
    {
        Arm::Slave::TrajPoint frame{};
        std::memcpy(&frame, data, kTrajFrameLen - 2);

        if (arm_slave == nullptr)
            return false;

        if (frame.index == 0 || !traj_base_valid)
        {
            arm_slave->clear();
            traj_base_valid = true;
        }

        last_vacuum_state = (frame.flags & kFlagVacuum) ? 1 : 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, static_cast<GPIO_PinState>(!last_vacuum_state));
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, static_cast<GPIO_PinState>(!last_vacuum_state));

        bool payload = (frame.flags & 0x40) ? 1 : 0;
        if (payload)
        {
            robot_arm->setPayload(0.6, 0.5);
        }
        else
            robot_arm->setPayload(0.0, 0.0);

        return arm_slave->pushPoint(frame, HAL_GetTick());
    }
};

static PcTrajRx  pc_traj_rx_obj(kUartPcHandler);
static PcTrajRx* pc_traj_rx = &pc_traj_rx_obj;
UartRxSync_DefineCallback(pc_traj_rx);

Arm::Controller::Config BuildArmConfig()
{
    Arm::Controller::Config cfg{};
    cfg.l1  = 0.340f;
    cfg.l2  = 0.380f;
    cfg.l3  = 0.1395f;
    cfg.lc1 = 0.2542f;
    cfg.lc2 = 0.28439f;
    cfg.lc3 = 0.0775f;
    cfg.m1  = 0.766f;
    cfg.m2  = 0.616f;
    cfg.m3  = 0.248f;
    cfg.g   = 9.81f;

    cfg.reduction_1 = 1.0f;
    cfg.reduction_2 = 100.0f * 187.0f / 3591.0f;
    cfg.reduction_3 = 1.0f;

    cfg.offset_1 = 0.0f;
    cfg.offset_2 = -163.87f;
    cfg.offset_3 = 0.0f;

    cfg.backlash_1 = 0.0f;
    cfg.backlash_2 = 4.0f;
    cfg.backlash_3 = 0.0f;
    return cfg;
}
} // namespace

void APP_ArmCtrl_BeforeUpdate()
{
    if (robot_arm != nullptr)
        return;
    if (motor_joint1 == nullptr || motor_joint2 == nullptr || motor_element == nullptr)
        return;

    static Arm::DMMITMotorCtrl  m1(motor_joint1);
    static Arm::DJIMITMotorCtrl m2(motor_joint2, kRatio2);
    static Arm::DMMITMotorCtrl  m3(motor_element);

    m1.SetMitParams(400.0f, 18.0f, 0.0f, 2.0f);
    m2.SetMitParams(0.8f, 0.03f, 0.01f, 1.0f);
    m3.SetMitParams(40.0f, 3.5f, 0.0f, 2.0f);

    joint1_motor = &m1;
    joint2_motor = &m2;
    joint3_motor = &m3;

    static Arm::Controller ctrl(*joint1_motor, *joint2_motor, *joint3_motor, BuildArmConfig());
    robot_arm = &ctrl;
    static Arm::Slave slave(ctrl);
    arm_slave = &slave;
    arm_slave->setTimeoutMs(200);
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

    if (arm_slave != nullptr)
        arm_slave->update(HAL_GetTick(), 0.001f);

    // robot_arm->update(kControlPeriod);
}

void APP_ArmCtrl_Update_200Hz() {}

void StatusFeedbackTask(void* argument)
{
    osDelay(4000);

    UartRxSync_RegisterCallback(pc_traj_rx, kUartPcHandler);
    pc_traj_rx->startReceive();

    for (;;)
    {
        if (robot_arm != nullptr)
        {
            float cur_q1  = 0.0f;
            float cur_q2  = 0.0f;
            float cur_q3  = 0.0f;
            float cur_dq1 = 0.0f;
            float cur_dq2 = 0.0f;
            float cur_dq3 = 0.0f;
            robot_arm->getJointAnglesComp(cur_q1, cur_q2, cur_q3);
            robot_arm->getJointVelocities(cur_dq1, cur_dq2, cur_dq3);

            const float pressure_kpa = APP_Device_GetPressureKpa();

            FeedbackFrame frame{};
            frame.sof          = kSof;
            frame.q1           = cur_q1;
            frame.q2           = cur_q2;
            frame.q3           = cur_q3;
            frame.dq1          = cur_dq1;
            frame.dq2          = cur_dq2;
            frame.dq3          = cur_dq3;
            frame.pressure_kpa = pressure_kpa;

            HAL_UART_Transmit_DMA(kUartPcHandler,
                                  reinterpret_cast<uint8_t*>(&frame),
                                  sizeof(frame));
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
