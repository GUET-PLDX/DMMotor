#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "../DMMotor.hpp"
#include "float_encoder.hpp"
#include "libxr_def.hpp"

int main() {
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
