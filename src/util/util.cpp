#include "hololib/util/util.hpp"
#include <numbers>

namespace hololib {

float getAngleError(float target, float current) { return std::remainder(target - current, 360.0f); }

float vexToMathRadians(float vexDegrees) {
    constexpr float TWO_PI = 2.0f * std::numbers::pi_v<float>;
    constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

    float radians = std::fmod((90.0f - vexDegrees) * DEG_TO_RAD, TWO_PI);
    return radians < 0.0f ? radians + TWO_PI : radians;
}
} // namespace hololib