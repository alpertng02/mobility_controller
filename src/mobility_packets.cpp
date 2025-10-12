#include "mobility_packets.hpp"

#include <stdexcept>
#include <cstring>

#include "mobility_packets.h"

std::vector<uint8_t> mobility::packets::
pack_motor_velocities(
    const std::array<float, k_motor_count>& velocities,
    uint64_t time_since_boot_us) {
    if (velocities.size() != k_motor_count) {
        throw std::invalid_argument("pack_motor_velocities: wrong number of velocities");
    }

    std::vector<uint8_t> buffer(
        mobility_packet_size_for_motor_velocities(), 0);

    if (!mobility_pack_motor_velocities_packet(buffer.data(), buffer.size(),
        time_since_boot_us,
        velocities.data()))
    {
        throw std::runtime_error("pack_motor_velocities: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::
pack_motor_duties(
    const std::array<float, k_motor_count>& duties,
    uint64_t time_since_boot_us) {
    if (duties.size() != k_motor_count) {
        throw std::invalid_argument("pack_motor_duties: wrong number of duties");
    }

    std::vector<uint8_t> buffer(
        mobility_packet_size_for_motor_duties(), 0);

    if (!mobility_pack_motor_dutycycles_packet(buffer.data(), buffer.size(),
        time_since_boot_us,
        duties.data()))
    {
        throw std::runtime_error("pack_motor_duties: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::
pack_motor_feedback(const Feedback& feedback, uint64_t time_since_boot_us) {

    std::vector<uint8_t> buffer(
        mobility_packet_size_for_feedback(), 0);

    if (!mobility_pack_feedback_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &feedback))
    {
        throw std::runtime_error("pack_motor_feedback: packet packing failed");
    }

    return buffer;
}


std::vector<uint8_t> mobility::packets::pack_device_state(const DeviceState& state, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_size_for_device_state(), 0);

    if (!mobility_pack_device_state_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &state))
    {
        throw std::runtime_error("pack_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::pack_running_mode_enable(const bool enable, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_enable_size(), 0);

    if (!mobility_pack_enable_packet(buffer.data(), buffer.size(),
        time_since_boot_us, mobility::packets::Commands::running_mode_enable, enable))
    {
        throw std::runtime_error("pack_running_mode_enable: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::pack_init_mode_enable(const bool enable, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_enable_size(), 0);

    if (!mobility_pack_enable_packet(buffer.data(), buffer.size(),
        time_since_boot_us, mobility::packets::Commands::init_mode_enable, enable))
    {
        throw std::runtime_error("pack_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::pack_request_device_state(uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_size_for_request(), 0);

    if (!mobility_pack_request_device_state_packet(buffer.data(), buffer.size(),
        time_since_boot_us))
    {
        throw std::runtime_error("pack_request_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> mobility::packets::pack_request_feedback(uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_size_for_request(), 0);

    if (!mobility_pack_request_feedback_packet(buffer.data(), buffer.size(),
        time_since_boot_us))
    {
        throw std::runtime_error("pack_request_feedback: packet packing failed");
    }

    return buffer;
}


std::vector<uint8_t> mobility::packets::
pack_init_packet(const InitPacket& init, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        mobility_packet_size_for_init(), 0);

    if (!mobility_pack_init_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &init))
    {
        throw std::runtime_error("pack_motor_duties: packet packing failed");
    }

    return buffer;
}


mobility::packets::
Header mobility::packets::
unpack_header(const std::vector<uint8_t>& buf) {
    if (buf.size() < mobility_packet_header_size()) {
        throw std::invalid_argument("unpack_header: buffer too small");
    }

    mobility::packets::
        Header hdr {};
    if (!mobility_unpack_packet_header(buf.data(), buf.size(), &hdr)) {
        throw std::runtime_error("unpack_header: unpacking unsuccesful");
    }

    return hdr;
}

mobility::packets::
Packet mobility::packets::
unpack_packet(const std::vector<uint8_t>& packet) {
    if (packet.empty())
        throw std::invalid_argument("unpack_packet: packet is empty");

    mobility::packets::
        Packet unpacked_packet {};

    unpacked_packet.header = unpack_header(packet);

    if (unpacked_packet.header.content_size <= 0) {
        throw std::runtime_error("unpack_packet: No content exists");
    }

    if (packet.size() < mobility_packet_header_size() + unpacked_packet.header.content_size)
        throw std::runtime_error("unpack_packet: buffer smaller than expected content size");

    unpacked_packet.payload.assign(
        packet.begin() + mobility_packet_header_size(),
        packet.begin() + mobility_packet_header_size() + unpacked_packet.header.content_size);

    return unpacked_packet;
};


std::array<float, mobility::packets::
    k_motor_count> mobility::packets::
    unpack_motor_velocities_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty())
        throw std::invalid_argument("unpack_motor_velocities_payload: payload is empty");

    std::array<float, mobility::packets::
        k_motor_count> velocities {};
    if (!mobility_unpack_motor_velocities_from_payload(
        payload.data(), payload.size(), velocities.data()))
    {
        throw std::runtime_error("unpack_motor_velocities_payload: failed to unpack payload");
    }
    return velocities;
};

std::array<float, mobility::packets::
    k_motor_count> mobility::packets::
    unpack_motor_duties_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty())
        throw std::invalid_argument("unpack_motor_duties_payload: payload empty");

    std::array<float, k_motor_count> duties {};
    if (!mobility_unpack_motor_dutycycles_from_payload(
        payload.data(), payload.size(), duties.data()))
    {
        throw std::runtime_error("unpack_motor_duties_payload: failed to unpack payload");
    }
    return duties;
}

mobility::packets::DeviceState
mobility::packets::unpack_device_state_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_device_state_payload: payload empty");
    }

    DeviceState state {};
    if (!mobility_unpack_device_state_packet_from_payload(payload.data(), payload.size(), &state)) {
        throw std::runtime_error("unpack_device_state_payload: failed to unpack payload");
    }
    return state;
}

mobility::packets::Feedback
mobility::packets::unpack_motor_feedback_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_motor_feedback_payload: payload empty");
    }

    Feedback feedback {};
    if (!mobility_unpack_feedback_packet_from_payload(payload.data(), payload.size(), &feedback)) {
        throw std::runtime_error("unpack_motor_feedback_payload: failed to unpack payload");
    }
    return feedback;
}

mobility::packets::InitPacket
mobility::packets::unpack_init_packet_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_init_packet_payload: payload empty");
    }

    InitPacket init {};
    if (!mobility_unpack_init_packet_from_payload(payload.data(), payload.size(), &init)) {
        throw std::runtime_error("unpack_init_packet_payload: failed to unpack payload");
    }
    return init;
}


