#include "armctrl.hpp"

#include "arm_controller.hpp"
#include "arm_motor_mit.hpp"
#include "arm_slave.hpp"
#include "cmsis_os2.h"
#include "crc.hpp"
#include "device.hpp"
#include "usart.h"
#include "UartRxSync.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

constexpr float kRatio2 = (3591.0f / (187.0f * 100.0f)) * (16384.0f / (20.0f * 0.3f));
// constexpr float kRatio3 = 16384.0f / (20.0f * 0.3f);

Arm::MITMotorCtrl* joint1_motor = nullptr;
Arm::MITMotorCtrl* joint2_motor = nullptr;
Arm::MITMotorCtrl* joint3_motor = nullptr;
Arm::Controller*   robot_arm    = nullptr;
Arm::Slave*        arm_slave    = nullptr;

constexpr UART_HandleTypeDef* kUartPcHandler = &huart1;
using CRC16Modbus                            = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint16_t kSof = 0xAA55;
constexpr size_t   kDebugTrajFrameLen = 43;

static bool traj_base_valid = false;

struct __attribute__((packed)) TrajRxFrame
{
    uint16_t              sof;
    Arm::Slave::TrajPoint point;
    uint16_t              crc16;
};


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
    uint16_t crc16;
};

constexpr size_t kTrajPayloadLen = sizeof(Arm::Slave::TrajPoint);
constexpr size_t kTrajFrameLen   = sizeof(TrajRxFrame);

static_assert(kTrajPayloadLen == 39, "Unexpected TrajPoint size");
static_assert(sizeof(TrajRxFrame) == sizeof(uint16_t) + kTrajPayloadLen + sizeof(uint16_t),
              "Unexpected trajectory frame size");
static_assert(sizeof(FeedbackFrame) == 32, "Unexpected feedback frame size");

static uint16_t ReadLE16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

extern "C"
{
volatile uint32_t g_pc_traj_rx_count          = 0;
volatile uint32_t g_pc_traj_decode_ok_count   = 0;
volatile uint32_t g_pc_traj_crc_fail_count    = 0;
volatile uint32_t g_pc_traj_push_fail_count   = 0;
volatile uint32_t g_pc_traj_last_tick_ms      = 0;
volatile uint16_t g_pc_traj_last_crc_rx       = 0;
volatile uint16_t g_pc_traj_last_crc_expected = 0;
volatile uint16_t g_pc_traj_last_index        = 0;
volatile uint8_t  g_pc_traj_last_flags        = 0;
volatile uint8_t  g_pc_traj_last_decode_ok    = 0;
volatile uint8_t  g_pc_traj_last_frame[kDebugTrajFrameLen] = {};
volatile float    g_pc_traj_last_q1           = 0.0f;
volatile float    g_pc_traj_last_q2           = 0.0f;
volatile float    g_pc_traj_last_q3           = 0.0f;
volatile float    g_pc_traj_last_dq1          = 0.0f;
volatile float    g_pc_traj_last_dq2          = 0.0f;
volatile float    g_pc_traj_last_dq3          = 0.0f;
volatile float    g_pc_traj_last_ddq1         = 0.0f;
volatile float    g_pc_traj_last_ddq2         = 0.0f;
volatile float    g_pc_traj_last_ddq3         = 0.0f;
}

static void DebugStoreLastFrame(const uint8_t* data)
{
    for (size_t i = 0; i < kTrajFrameLen && i < kDebugTrajFrameLen; ++i)
        g_pc_traj_last_frame[i] = data[i];
}

static void DebugStoreLastPoint(const Arm::Slave::TrajPoint& point)
{
    g_pc_traj_last_index = point.index;
    g_pc_traj_last_q1    = point.q1;
    g_pc_traj_last_q2    = point.q2;
    g_pc_traj_last_q3    = point.q3;
    g_pc_traj_last_dq1   = point.dq1;
    g_pc_traj_last_dq2   = point.dq2;
    g_pc_traj_last_dq3   = point.dq3;
    g_pc_traj_last_ddq1  = point.ddq1;
    g_pc_traj_last_ddq2  = point.ddq2;
    g_pc_traj_last_ddq3  = point.ddq3;
    g_pc_traj_last_flags = point.flags;
}

/**
 * PC Trajectory Receiver:
 * - 接收来自上位机的轨迹点，解析后通过 `arm_slave` 下发给控制器。
 * - flags 随轨迹点进入队列，在轨迹点实际执行时生效。
 *
 * 帧结构（共 41 字节）：
 * - SOF (2 bytes): 0x55AA
 * - index (2 bytes): 轨迹点序号
 * - q1, q2, q3 (4 bytes each): 目标关节角度，单位为度
 * - dq1, dq2, dq3 (4 bytes each): 目标关节角速度，单位为度/s
 * - ddq1, ddq2, ddq3 (4 bytes each): 目标关节角加速度，单位为度/s^2
 * - flags (1 byte): bit7-吸盘状态，bit6-末端补偿状态
 *
 * 注意：这个模块直接操作全局指针 `arm_slave` 和 `robot_arm`，
 * 因此必须确保它们在使用前已经正确初始化。
 */
class PcTrajRx final : public protocol::UartRxSync<2, kTrajFrameLen, true>
{
public:
    explicit PcTrajRx(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

protected:
    const std::array<uint8_t, 2>& header() const override
    {
        static constexpr std::array<uint8_t, 2> kHeader = { 0x55, 0xAA };
        return kHeader;
    }

    bool decode(const uint8_t data[kTrajFrameLen]) override
    {
        ++g_pc_traj_rx_count;
        g_pc_traj_last_tick_ms = HAL_GetTick();
        DebugStoreLastFrame(data);

        const uint16_t rx_crc       = ReadLE16(data + kTrajFrameLen - sizeof(uint16_t));
        const uint16_t expected_crc = CRC16Modbus::calc(data, kTrajFrameLen - sizeof(uint16_t));
        g_pc_traj_last_crc_rx       = rx_crc;
        g_pc_traj_last_crc_expected = expected_crc;
        if (rx_crc != expected_crc)
        {
            ++g_pc_traj_crc_fail_count;
            g_pc_traj_last_decode_ok = 0;
            return false;
        }

        TrajRxFrame frame{};
        std::memcpy(&frame, data, sizeof(frame));
        DebugStoreLastPoint(frame.point);

        if (arm_slave == nullptr)
        {
            g_pc_traj_last_decode_ok = 0;
            return false;
        }

        if (frame.point.index == 0 || !traj_base_valid)
        {
            arm_slave->clear();
            traj_base_valid = true;
        }

        const bool ok = arm_slave->pushPoint(frame.point, HAL_GetTick());
        if (ok)
        {
            ++g_pc_traj_decode_ok_count;
            g_pc_traj_last_decode_ok = 1;
        }
        else
        {
            ++g_pc_traj_push_fail_count;
            g_pc_traj_last_decode_ok = 0;
        }
        return ok;
    }
};

 PcTrajRx  pc_traj_rx_obj(kUartPcHandler);
 PcTrajRx* pc_traj_rx = &pc_traj_rx_obj;
UartRxSync_DefineCallback(pc_traj_rx);

Arm::Controller::Config BuildArmConfig()
{
    Arm::Controller::Config cfg{};
    cfg.l1  = 0.360f;
    cfg.l2  = 0.380f;
    cfg.l3  = 0.1395f;
    cfg.lc1 = 0.290f;
    cfg.lc2 = 0.270231f;
    cfg.lc3 = 0.0775f;
    cfg.m1  = 0.708f;
    cfg.m2  = 0.498f;
    cfg.m3  = 0.248f;

    cfg.I1 = 0.006193425f;
    cfg.I2 = 0.008844550f;
    cfg.I3 = 0.000279927f;

    cfg.m_payload  = 0.6;
    cfg.lc_payload = 0.175;
    cfg.I_payload  = 0.012217;

    cfg.g = 9.81f;

    cfg.reduction_1 = 1.0f;
    cfg.reduction_2 = 100.0f * 187.0f / 3591.0f;
    cfg.reduction_3 = 1.0f;

    cfg.offset_1 = 0.0f;
    cfg.offset_2 = -165.73f;
    cfg.offset_3 = 0.0f;

    cfg.backlash_1 = 0.0f;
    cfg.backlash_2 = 7.0f;
    cfg.backlash_3 = 0.0f;
    return cfg;
}

void APP_ArmCtrl_BeforeUpdate()
{
    if (robot_arm != nullptr)
        return;
    if (motor_joint1 == nullptr || motor_joint2 == nullptr || motor_element == nullptr)
        return;

    static Arm::DMMITMotorCtrl  m1(motor_joint1);
    static Arm::DJIMITMotorCtrl m2(motor_joint2, kRatio2);
    static Arm::DMMITMotorCtrl  m3(motor_element);

    m1.SetMitParams(300.0f, 18.0f, 0.0f, 2.0f);
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

            static FeedbackFrame frame{};
            frame.sof          = kSof;
            frame.q1           = cur_q1;
            frame.q2           = cur_q2;
            frame.q3           = cur_q3;
            frame.dq1          = cur_dq1;
            frame.dq2          = cur_dq2;
            frame.dq3          = cur_dq3;
            frame.pressure_kpa = pressure_kpa;
            frame.crc16        = CRC16Modbus::calc(reinterpret_cast<const uint8_t*>(&frame),
                                                   sizeof(frame) - sizeof(frame.crc16));

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
