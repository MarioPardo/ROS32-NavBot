#include "ros32bot_middleware/package_bytes.hpp"

namespace ros32bot_middleware
{
    namespace
    {
        constexpr uint8_t SYNC_0 = 0xA5;
        constexpr uint8_t SYNC_1 = 0x5A;

        // Must match uart_comms.h
        constexpr uint8_t MSG_SET_WHEEL_VEL = 0x01;

        // CCITT-FALSE. Same polynomial and seed as the STM32 side.
        uint16_t crc16Ccitt(const Bytes & body)
        {
            uint16_t crc = 0xFFFF;

            for (uint8_t byte : body)
            {
                crc ^= static_cast<uint16_t>(byte) << 8;

                for (int bit = 0; bit < 8; bit++)
                {
                    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                         : static_cast<uint16_t>(crc << 1);
                }
            }

            return crc;
        }

        void appendInt16(Bytes & out, int16_t value)
        {
            out.push_back(static_cast<uint8_t>(value & 0xFF));
            out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        }

        // Wraps id, seq, len and payload in sync bytes and a trailing crc.
        Bytes makeFrame(uint8_t msg_id, const Bytes & payload)
        {
            static uint8_t seq = 0;

            Bytes body;
            body.push_back(msg_id);
            body.push_back(seq++);
            body.push_back(static_cast<uint8_t>(payload.size()));
            body.insert(body.end(), payload.begin(), payload.end());

            const uint16_t crc = crc16Ccitt(body);

            Bytes frame;
            frame.reserve(body.size() + 4);
            frame.push_back(SYNC_0);
            frame.push_back(SYNC_1);
            frame.insert(frame.end(), body.begin(), body.end());
            frame.push_back(static_cast<uint8_t>(crc & 0xFF));
            frame.push_back(static_cast<uint8_t>(crc >> 8));

            return frame;
        }
    }

    Bytes packageWheelVelocities(int16_t left_crad_s, int16_t right_crad_s)
    {
        Bytes payload;
        appendInt16(payload, left_crad_s);
        appendInt16(payload, right_crad_s);

        return makeFrame(MSG_SET_WHEEL_VEL, payload);
    }
}
