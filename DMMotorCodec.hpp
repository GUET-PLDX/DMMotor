#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

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

inline float UintToFloat(uint16_t value, float minimum, float maximum,
                         uint8_t bits) {
  const uint32_t FULL_SCALE = (uint32_t{1} << bits) - 1U;
  return minimum + static_cast<float>(value) * (maximum - minimum) /
                       static_cast<float>(FULL_SCALE);
}

inline uint16_t FloatToUintClamped(float value, float minimum, float maximum,
                                   uint8_t bits) {
  // MITControl validates the complete command before calling this helper.
  const float CLAMPED = std::clamp(value, minimum, maximum);
  const uint32_t FULL_SCALE = (uint32_t{1} << bits) - 1U;
  return static_cast<uint16_t>((CLAMPED - minimum) *
                               static_cast<float>(FULL_SCALE) /
                               (maximum - minimum));
}
}  // namespace DMMotorCodec
