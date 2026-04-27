#pragma once

#include <cstdint>

namespace mathUtils
{
    double degToRad(double degrees);

    std::uint32_t rotateLeft(std::uint32_t x, int k);

    double randomDouble(std::uint32_t state[4], const double scale);
}