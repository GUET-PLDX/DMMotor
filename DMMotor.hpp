#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - param:
      model: DMMotor::Model::MOTOR_DM4310
      reverse: false
      can_id: 1
      can_bus_name: can1
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "Motor.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "float_encoder.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"

namespace DMMotorCodec {
struct MitCommand {
  float pos;
  float vel;
  float kp;
  float kd;
  float tor;
};

inline MitCommand NormalizeMitCommand(MitCommand command) {
  if (!std::isfinite(command.pos) || !std::isfinite(command.vel) ||
      !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
      !std::isfinite(command.tor)) {
    return {};
  }
  return command;
}

template <int Bits>
inline float UintToFloat(uint16_t value, float minimum, float maximum) {
  return LibXR::FloatEncoder<Bits>(minimum, maximum).Decode(value);
}

template <int Bits>
inline uint16_t FloatToUintClamped(float value, float minimum, float maximum) {
  return static_cast<uint16_t>(
      LibXR::FloatEncoder<Bits>(minimum, maximum).Encode(value));
}
}  // namespace DMMotorCodec

#define DM4310_PMAX (static_cast<float>(LibXR::TWO_PI))
#define DM4310_VMAX (30.0f)
#define DM4310_TMAX (10.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)

#define DM_MOTOR_STATE_DISABLED (0x0u)
#define DM_MOTOR_STATE_ENABLED (0x1u)

#define DM8009_PMAX (static_cast<float>(2.0 * LibXR::TWO_PI))
#define DM8009_VMAX (45.0f)
#define DM8009_TMAX (54.0f)
#define DM8009_KP_MIN (0.0f)
#define DM8009_KP_MAX (500.0f)
#define DM8009_KD_MIN (0.0f)
#define DM8009_KD_MAX (5.0f)

class DMMotor : public LibXR::Application, public Motor {
 public:
  /*电机型号*/
  enum class Model : uint8_t {
    MOTOR_NONE = 0,
    MOTOR_DM4310,
    MOTOR_DM8009,
  };

  /*电机参数*/
  struct Param {
    Model model;
    bool reverse;
    uint16_t can_id;
    const char* can_bus_name;
  };

  /*量程*/
  struct LSB {
    float P_MAX;
    float V_MAX;
    float T_MAX;
    float KD_MIN;
    float KD_MAX;
    float KP_MIN;
    float KP_MAX;
  };

  /**
   * @brief DMMotor 的构造函数
   * @param hw
   * @param app
   * @param param 电机参数 (电机型号 是否反转 CANID CanBusName)
   */
  DMMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const Param& param)
      : param_(param),
        feedback_{},
        startup_time_(LibXR::Timebase::GetMicroseconds()),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);

    switch (param_.model) {
      case Model::MOTOR_DM4310:
        lsb_.P_MAX = DM4310_PMAX;
        lsb_.V_MAX = DM4310_VMAX;
        lsb_.T_MAX = DM4310_TMAX;
        lsb_.KD_MIN = DM4310_KD_MIN;
        lsb_.KD_MAX = DM4310_KD_MAX;
        lsb_.KP_MIN = DM4310_KP_MIN;
        lsb_.KP_MAX = DM4310_KP_MAX;
        break;
      case Model::MOTOR_DM8009:
        lsb_.P_MAX = DM8009_PMAX;
        lsb_.V_MAX = DM8009_VMAX;
        lsb_.T_MAX = DM8009_TMAX;
        lsb_.KD_MIN = DM8009_KD_MIN;
        lsb_.KD_MAX = DM8009_KD_MAX;
        lsb_.KP_MIN = DM8009_KP_MIN;
        lsb_.KP_MAX = DM8009_KP_MAX;
        break;
      case Model::MOTOR_NONE:
        lsb_.P_MAX = 0;
        lsb_.V_MAX = 0;
        lsb_.T_MAX = 0;
        lsb_.KD_MIN = 0;
        lsb_.KD_MAX = 0;
        lsb_.KP_MIN = 0;
        lsb_.KP_MAX = 0;
        break;
    }
    /* 强制规定达妙电机反馈id=自身id+10 */
    uint16_t feedback_id_to_register = 0x10 + param_.can_id;

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);
    /* 注册can */
    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, feedback_id_to_register,
                   feedback_id_to_register);
  }

  /*使能*/
  void Enable() override {
    /*使能can包*/
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /*失能*/
  void Disable() override {
    /*失能can包*/
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void Relax() override { Disable(); }

  LibXR::ErrorCode Update() override {
    const auto NOW = LibXR::Timebase::GetMicroseconds();
    bool get_feedback = false;
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == LibXR::ErrorCode::OK) {
      Decode(pack);
      get_feedback = true;
    }

    if (get_feedback) {
      feedback_received_ = true;
      last_online_time_ = NOW;
      return LibXR::ErrorCode::OK;
    }

    const auto AGE =
        feedback_received_ ? NOW - last_online_time_ : NOW - startup_time_;
    const uint64_t TIMEOUT =
        feedback_received_ ? FEEDBACK_TIMEOUT_US : STARTUP_GRACE_US;
    return AGE.ToMicrosecond() <= TIMEOUT ? LibXR::ErrorCode::OK
                                          : LibXR::ErrorCode::NO_RESPONSE;
  }

  const Feedback& GetFeedback() override { return feedback_; }

  void Control(const MotorCmd& cmd) override {
    switch (cmd.mode) {
      case ControlMode::MODE_POSITION:
        PosControl(cmd.position, cmd.velocity);
        break;
      case ControlMode::MODE_VELOCITY:
        SpdControl(cmd.velocity);
        break;
      case ControlMode::MODE_TORQUE:
        MITControl(0.0f, 0.0f, 0.0f, 0.0f, cmd.torque);
        break;
      case ControlMode::MODE_MIT:
        MITControl(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
        break;
      default:
        break;
    }
  }

  /*重置错误状态*/
  void ClearError() override {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /*将当前位置设成零点*/
  void SaveZeroPoint() override {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void OnMonitor() override {}

 private:
  static constexpr uint64_t STARTUP_GRACE_US = 200000U;
  static constexpr uint64_t FEEDBACK_TIMEOUT_US = 150000U;
  Param param_;
  LSB lsb_;
  Motor::Feedback feedback_;
  LibXR::MicrosecondTimestamp startup_time_{};
  LibXR::MicrosecondTimestamp last_online_time_{};
  bool feedback_received_ = false;
  LibXR::CAN* can_;
  LibXR::MPMCQueue<LibXR::CAN::ClassicPack> recv_queue_{2};

  /**
   * @brief CAN 接收回调的静态包装函数
   * @details
   * 将接收到的CAN数据包推入无锁队列中，供后续处理。如果队列已满，则丢弃最旧的数据包。
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self 用户提供的参数，这里是 RMMotorContainer 实例的指针
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, DMMotor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != LibXR::ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

  void Decode(LibXR::CAN::ClassicPack& pack) {
    const uint8_t motor_state = (pack.data[0] >> 4) & 0x0F;
    feedback_.state = motor_state;
    feedback_.error_id = motor_state == DM_MOTOR_STATE_DISABLED ||
                                 motor_state == DM_MOTOR_STATE_ENABLED
                             ? 0
                             : motor_state;
    const uint16_t POSITION_RAW =
        (static_cast<uint16_t>(pack.data[1]) << 8U) | pack.data[2];
    feedback_.position =
        DMMotorCodec::UintToFloat<16>(POSITION_RAW, -lsb_.P_MAX, lsb_.P_MAX);

    const uint16_t VELOCITY_RAW =
        (static_cast<uint16_t>(pack.data[3]) << 4U) | (pack.data[4] >> 4U);
    feedback_.omega =
        DMMotorCodec::UintToFloat<12>(VELOCITY_RAW, -lsb_.V_MAX, lsb_.V_MAX);
    feedback_.velocity =
        feedback_.omega * 60.0f / static_cast<float>(LibXR::TWO_PI);
    const uint16_t TORQUE_RAW =
        (static_cast<uint16_t>(pack.data[4] & 0xFU) << 8U) | pack.data[5];
    feedback_.torque =
        DMMotorCodec::UintToFloat<12>(TORQUE_RAW, -lsb_.T_MAX, lsb_.T_MAX);
    feedback_.temp = static_cast<float>(
        pack.data[6] > pack.data[7] ? pack.data[6] : pack.data[7]);

    if (param_.reverse) {
      feedback_.position = -feedback_.position;
      feedback_.velocity = -feedback_.velocity;
      feedback_.torque = -feedback_.torque;
      feedback_.omega = -feedback_.omega;
    }
    feedback_.abs_angle = feedback_.position;
  }

 public:
  float GetAngle() const { return feedback_.position; }
  float GetTor() const { return feedback_.torque; }
  float GetOmega() const { return feedback_.omega; }

  void MITControl(float pos, float vel, float kp, float kd, float tor) {
    if (this->feedback_.temp > 90.0f) {
      Disable();
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
      return;
    }

    const auto COMMAND = DMMotorCodec::NormalizeMitCommand(
        {.pos = pos, .vel = vel, .kp = kp, .kd = kd, .tor = tor});
    pos = COMMAND.pos;
    vel = COMMAND.vel;
    kp = COMMAND.kp;
    kd = COMMAND.kd;
    tor = COMMAND.tor;

    const float SEND_POS = param_.reverse ? -pos : pos;
    const float SEND_VEL = param_.reverse ? -vel : vel;
    const float SEND_TOR = param_.reverse ? -tor : tor;

    const uint16_t POS_U =
        DMMotorCodec::FloatToUintClamped<16>(SEND_POS, -lsb_.P_MAX, lsb_.P_MAX);
    const uint16_t VEL_U =
        DMMotorCodec::FloatToUintClamped<12>(SEND_VEL, -lsb_.V_MAX, lsb_.V_MAX);
    const uint16_t TOR_U =
        DMMotorCodec::FloatToUintClamped<12>(SEND_TOR, -lsb_.T_MAX, lsb_.T_MAX);
    const uint16_t KP_U =
        DMMotorCodec::FloatToUintClamped<12>(kp, lsb_.KP_MIN, lsb_.KP_MAX);
    const uint16_t KD_U =
        DMMotorCodec::FloatToUintClamped<12>(kd, lsb_.KD_MIN, lsb_.KD_MAX);

    uint8_t data[8];
    data[0] = (POS_U >> 8) & 0xFF;
    data[1] = POS_U & 0xFF;
    data[2] = (VEL_U >> 4) & 0xFF;
    data[3] = ((VEL_U & 0xF) << 4) | ((KP_U >> 8) & 0xF);
    data[4] = KP_U & 0xFF;
    data[5] = (KD_U >> 4) & 0xFF;
    data[6] = ((KD_U & 0xF) << 4) | ((TOR_U >> 8) & 0xF);
    data[7] = TOR_U & 0xFF;

    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

 private:
  void PosControl(float pos, float vel) {
    if (this->feedback_.temp > 90.0f) {
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
      Disable();
      return;
    }
    pos = std::clamp(pos, -lsb_.P_MAX, lsb_.P_MAX);
    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);

    float send_pos = param_.reverse ? -pos : pos;
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[8];
    uint8_t* pbuf = reinterpret_cast<uint8_t*>(&send_pos);
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i) {
      data[i] = pbuf[i];
      data[i + 4] = vbuf[i];
    }

    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void SpdControl(float vel) {
    if (this->feedback_.temp > 85.0f) {
      Disable();
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
      return;
    }

    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[4];
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i) {
      data[i] = vbuf[i];
    }

    uint32_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data, data, 4);
    can_->AddMessage(tx_pack);
  }
};
