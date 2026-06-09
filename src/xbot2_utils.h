#ifndef XBOT2_MUJOCO_UTILS_H
#define XBOT2_MUJOCO_UTILS_H

#include <cstdint>
#include <cstring>
#include <utility>

namespace XBot::utils {

void patch_double(double& value, uint32_t stamp_us)
{
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    bits = (bits & ~uint64_t(0xFFFFF)) | (stamp_us & 0xFFFFFu);
    std::memcpy(&value, &bits, sizeof(value));
}

std::pair<int32_t, uint32_t> decode_stamp_and_delay(double value, uint32_t now_us)
{
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    uint32_t stamp_us = static_cast<uint32_t>(bits & 0xFFFFFu);
    // handle wraparound (period = 2^20 us ~ 1.05 s)
    int32_t delay_us = static_cast<int32_t>((now_us - stamp_us) & 0xFFFFFu);
    if(delay_us > (1 << 19)) delay_us -= (1 << 20);  // signed correction
    return {stamp_us, delay_us};
}

}

#endif // XBOT2_MUJOCO_UTILS_H
