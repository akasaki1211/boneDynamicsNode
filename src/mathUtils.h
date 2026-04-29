#pragma once

#include <cstdint>

namespace mathUtils
{
    constexpr double kPi = 3.141592653589793238462643383279502884;

    double degToRad(double degrees);

    std::uint32_t rotateLeft(std::uint32_t x, int k);

    double randomDouble(std::uint32_t state[4], const double scale);
}