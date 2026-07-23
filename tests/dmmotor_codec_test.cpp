#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "../DMMotorCodec.hpp"

int main() {
  constexpr float PMAX = 6.283185f;
  assert(std::fabs(DMMotorCodec::UintToFloat(0x0000U, -PMAX, PMAX, 16) + PMAX) <
         1e-5f);
  assert(std::fabs(DMMotorCodec::UintToFloat(0x8000U, -PMAX, PMAX, 16)) <
         2e-4f);
  assert(std::fabs(DMMotorCodec::UintToFloat(0xffffU, -PMAX, PMAX, 16) - PMAX) <
         1e-5f);
  assert(DMMotorCodec::FloatToUintClamped(-1.0f, 0.0f, 500.0f, 12) == 0U);
  assert(DMMotorCodec::FloatToUintClamped(600.0f, 0.0f, 500.0f, 12) == 0xfffU);
  assert(DMMotorCodec::FloatToUintClamped(0.0f, 0.0f, 5.0f, 12) == 0U);

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
