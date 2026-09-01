#pragma once

#include <cmath>
#include <cstdint>

#include "float_encoder.hpp"

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
