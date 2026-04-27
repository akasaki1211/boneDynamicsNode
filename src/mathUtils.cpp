#include "mathUtils.h"

namespace mathUtils
{
    constexpr double kPi = 3.141592653589793238462643383279502884;

    double degToRad(double degrees)
    {
        return degrees * (kPi / 180.0);
    }

    std::uint32_t rotateLeft(std::uint32_t x, int k)
    {
        return (x << k) | (x >> (32 - k));
    }

    double randomDouble(std::uint32_t state[4], const double scale)
    {
        // xoshiro128++
        std::uint32_t result = rotateLeft(state[0] + state[3], 7) + state[0];

        std::uint32_t t = state[1] << 9;

        state[2] ^= state[0];
        state[3] ^= state[1];
        state[1] ^= state[2];
        state[0] ^= state[3];

        state[2] ^= t;

        state[3] = rotateLeft(state[3], 11);

        // normalize
        const double normalized = result * 2.3283064365386963e-10;

        // 0~1 to -scale ~ +scale.
        return normalized * (scale * 2.0) - scale;
    }
}