#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "../DMMotor.hpp"
#include "float_encoder.hpp"
#include "libxr_def.hpp"

int main() {
  const auto MODE_WRITE = DMMotorCodec::MakeParameterWriteFrame(
      0x0123U, DMMotorCodec::CONTROL_MODE_REGISTER,
      DMMotorCodec::POSITION_VELOCITY_MODE);
  assert(MODE_WRITE.id == 0x7FFU);
  assert(MODE_WRITE.type == LibXR::CAN::Type::STANDARD && MODE_WRITE.dlc == 8);
  assert(MODE_WRITE.data[0] == 0x23U && MODE_WRITE.data[1] == 0x01U);
  assert(MODE_WRITE.data[2] == 0x55U && MODE_WRITE.data[3] == 0x0AU);
  assert(MODE_WRITE.data[4] == 2U);
  for (int i = 5; i < 8; ++i) assert(MODE_WRITE.data[i] == 0U);

  const auto SAVE = DMMotorCodec::MakeSaveParametersFrame(0x0123U);
  assert(SAVE.id == 0x7FFU);
  assert(SAVE.type == LibXR::CAN::Type::STANDARD && SAVE.dlc == 8);
  assert(SAVE.data[0] == 0x23U && SAVE.data[1] == 0x01U);
  assert(SAVE.data[2] == 0xAAU && SAVE.data[3] == 0x01U);
  for (int i = 4; i < 8; ++i) assert(SAVE.data[i] == 0U);

  const float target_position = 1.25f;
  const float max_velocity = 2.5f;
  const auto POSITION_VELOCITY = DMMotorCodec::MakePositionVelocityFrame(
      0x0123U, target_position, max_velocity);
  assert(POSITION_VELOCITY.id == 0x223U);
  assert(POSITION_VELOCITY.type == LibXR::CAN::Type::STANDARD &&
         POSITION_VELOCITY.dlc == 8);
  const uint8_t expected_position[] = {0x00U, 0x00U, 0xA0U, 0x3FU};
  const uint8_t expected_velocity[] = {0x00U, 0x00U, 0x20U, 0x40U};
  for (int i = 0; i < 4; ++i) {
    assert(POSITION_VELOCITY.data[i] == expected_position[i]);
    assert(POSITION_VELOCITY.data[i + 4] == expected_velocity[i]);
  }

  LibXR::CAN::ClassicPack frame{};
  frame.type = LibXR::CAN::Type::STANDARD;
  frame.dlc = 8;
  assert(DMMotorCodec::IsFeedbackFrame(frame));
  frame.dlc = 7;
  assert(!DMMotorCodec::IsFeedbackFrame(frame));
  frame.dlc = 8;
  frame.type = LibXR::CAN::Type::REMOTE_STANDARD;
  assert(!DMMotorCodec::IsFeedbackFrame(frame));

  constexpr float PMAX = static_cast<float>(LibXR::TWO_PI);
  assert(std::fabs(DMMotorCodec::UintToFloat<16>(0x0000U, -PMAX, PMAX) + PMAX) <
         1e-5f);
  assert(std::fabs(DMMotorCodec::UintToFloat<16>(0x8000U, -PMAX, PMAX)) <
         2e-4f);
  assert(std::fabs(DMMotorCodec::UintToFloat<16>(0xffffU, -PMAX, PMAX) - PMAX) <
         1e-5f);
  assert(DMMotorCodec::FloatToUintClamped<12>(-1.0f, 0.0f, 500.0f) == 0U);
  assert(DMMotorCodec::FloatToUintClamped<12>(600.0f, 0.0f, 500.0f) == 0xfffU);
  assert(DMMotorCodec::FloatToUintClamped<12>(0.0f, 0.0f, 5.0f) == 0U);

  const LibXR::FloatEncoder<12> encoder(0.0f, 5.0f);
  assert(DMMotorCodec::FloatToUintClamped<12>(2.5f, 0.0f, 5.0f) ==
         encoder.Encode(2.5f));
  assert(std::fabs(
             DMMotorCodec::UintToFloat<12>(encoder.Encode(2.5f), 0.0f, 5.0f) -
             encoder.Decode(encoder.Encode(2.5f))) < 1e-6f);

  constexpr DMMotorCodec::MitCommand VALID = {
      .pos = 1.0f, .vel = 2.0f, .kp = 3.0f, .kd = 4.0f, .tor = 5.0f};
  constexpr float NAN_VALUE = std::numeric_limits<float>::quiet_NaN();
  constexpr float INF_VALUE = std::numeric_limits<float>::infinity();
  const DMMotorCodec::MitCommand INVALID_COMMANDS[] = {
      {.pos = NAN_VALUE,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = INF_VALUE,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = NAN_VALUE,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = -INF_VALUE,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = NAN_VALUE,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = INF_VALUE,
       .kd = VALID.kd,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = NAN_VALUE,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = -INF_VALUE,
       .tor = VALID.tor},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = NAN_VALUE},
      {.pos = VALID.pos,
       .vel = VALID.vel,
       .kp = VALID.kp,
       .kd = VALID.kd,
       .tor = INF_VALUE},
  };

  for (const auto& INVALID : INVALID_COMMANDS) {
    const auto NEUTRAL = DMMotorCodec::NormalizeMitCommand(INVALID);
    assert(NEUTRAL.pos == 0.0f && NEUTRAL.vel == 0.0f && NEUTRAL.kp == 0.0f &&
           NEUTRAL.kd == 0.0f && NEUTRAL.tor == 0.0f);
  }
}
