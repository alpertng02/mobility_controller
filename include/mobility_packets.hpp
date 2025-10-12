#ifndef __MOBILITY_PACKETS_HPP__
#define __MOBILITY_PACKETS_HPP__

#include "mobility_packets.h"

#include <vector>
#include <array>
#include <cstdint>

namespace mobility {

namespace packets {

constexpr uint32_t k_device_id = MOBILITY_DEVICE_ID;
constexpr size_t k_motor_count = MOBILITY_MOTOR_COUNT;

using Header = MobilityPacketHeader;
using InitPacket = MobilityInitPacket;
using Commands = MobilityCommands;
using DeviceState = MobilityDeviceState;
using Feedback = MobilityFeedback;

struct Packet {
    Header header;
    std::vector<uint8_t> payload;
};

struct FeedbackPacket {
    Header header;
    Feedback feedback;
};

struct DeviceStatePacket {
    Header header;
    DeviceState state;
};

// Packing

std::vector<uint8_t> pack_motor_velocities(const std::array<float, k_motor_count>& velocities,
    uint64_t time_since_boot_us);

std::vector<uint8_t> pack_motor_duties(const std::array<float, k_motor_count>& duties,
    uint64_t time_since_boot_us);

std::vector<uint8_t> pack_motor_feedback(const Feedback& feedback, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_device_state(const DeviceState& state, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_request_device_state(uint64_t time_since_boot_us);

std::vector<uint8_t> pack_request_feedback(uint64_t time_since_boot_us);

std::vector<uint8_t> pack_init_packet(const InitPacket& init, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_init_mode_enable(const bool enable, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_running_mode_enable(const bool enable, uint64_t time_since_boot_us);


// Unpacking

Header unpack_header(const std::vector<uint8_t>& buf);

Packet unpack_packet(const std::vector<uint8_t>& packet);

std::array<float, k_motor_count> unpack_motor_velocities_payload(const std::vector<uint8_t>& payload);

Feedback unpack_motor_feedback_payload(const std::vector<uint8_t>& payload);

std::array<float, k_motor_count> unpack_motor_duties_payload(const std::vector<uint8_t>& payload);

DeviceState unpack_device_state_payload(const std::vector<uint8_t>& payload);

InitPacket unpack_init_packet_payload(const std::vector<uint8_t>& payload);

}

}


#endif // __MOBILITY_PACKETS_HPP__