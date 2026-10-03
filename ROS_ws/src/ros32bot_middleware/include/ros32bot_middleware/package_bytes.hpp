#ifndef ROS32BOT_MIDDLEWARE_PACKAGE_BYTES_HPP
#define ROS32BOT_MIDDLEWARE_PACKAGE_BYTES_HPP

#include <cstdint>
#include <vector>

namespace ros32bot_middleware
{
    using Bytes = std::vector<uint8_t>;

    // Frame layout must stay in step with STM32/UART_PROTOCOL.md and uart_comms.h
    // Wheel velocity targets in centirad/s, left then right.
    Bytes packageWheelVelocities(int16_t left_crad_s, int16_t right_crad_s);
}

#endif
