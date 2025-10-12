/**
 *@file icommunication_protocol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "imobility_device.hpp"
#include "mobility_packets.hpp"
#include "mobility_packets.h"
#include <chrono>
#include <thread>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <iostream>


IMobilityDevice::~IMobilityDevice() = default;


bool IMobilityDevice::send_motor_velocities(const std::array<float, mobility::packets::k_motor_count>& velocities) {
    try {
        auto packet = mobility::packets::pack_motor_velocities(velocities, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IMobilityDevice::send_motor_dutycycles(const std::array<float, mobility::packets::k_motor_count>& dutycycles) {
    try {
        auto packet = mobility::packets::pack_motor_duties(dutycycles, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IMobilityDevice::send_init_packet(const mobility::packets::InitPacket& init_packet) {
    try {
        auto packet = mobility::packets::pack_init_packet(init_packet, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IMobilityDevice::send_init_mode_enable(const bool enable) {
    try {
        auto packet = mobility::packets::pack_init_mode_enable(enable, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IMobilityDevice::send_running_mode_enable(const bool enable) {
    try {
        auto packet = mobility::packets::pack_running_mode_enable(enable, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}


bool IMobilityDevice::init_device(const mobility::packets::InitPacket& init_packet, std::chrono::milliseconds timeout) {
    try {

        auto start_time = std::chrono::steady_clock::now();
        auto device_state = this->receive_device_state(std::chrono::milliseconds(5));
        while (device_state.state.device_is_init) {
            if (this->send_init_mode_enable(true)) {
                device_state = this->receive_device_state(std::chrono::milliseconds(5));
            }
            auto current_time = std::chrono::steady_clock::now();
            auto total_time = current_time - start_time;
            if (total_time > timeout) {
                return false;
            }
        }

        if (!this->send_init_packet(init_packet)) {
            return false;
        }

        device_state = this->receive_device_state(std::chrono::milliseconds(5));
        if (device_state.state.device_is_init && device_state.state.device_is_running) {
            return true;
        }
    } catch (std::runtime_error& err) {
        return false;
    }
    return false;
}


mobility::packets::FeedbackPacket IMobilityDevice::receive_motor_feedback(std::chrono::milliseconds timeout) {
    auto request_packet = mobility::packets::pack_request_feedback(0);
    int bytes_sent = this->write_bytes(request_packet.data(), request_packet.size());
    if (bytes_sent < 0) {
        throw std::runtime_error("receive_motor_feedback: could not send request packet");
    }

    if (static_cast<size_t>(bytes_sent) != request_packet.size()) {
        throw std::runtime_error("receive_motor_feedback: could not send all bytes of packet");
    }

    auto packet = this->receive_packet(timeout);
    auto unpacked = mobility::packets::unpack_packet(packet);

    if (unpacked.header.command_id == mobility::packets::Commands::motor_feedback) {
        mobility::packets::FeedbackPacket feedback = {
            .header = unpacked.header,
            .feedback = mobility::packets::Feedback()
        };
        feedback.feedback = mobility::packets::unpack_motor_feedback_payload(unpacked.payload);
        return feedback;
    } else {
        throw std::runtime_error("receive_motor_feedback: package received is not package");
    }

}

mobility::packets::DeviceStatePacket IMobilityDevice::receive_device_state(std::chrono::milliseconds timeout) {
    auto request_packet = mobility::packets::pack_request_device_state(0);
    int bytes_sent = this->write_bytes(request_packet.data(), request_packet.size());
    if (bytes_sent < 0) {
        throw std::runtime_error("receive_device_state: could not send request packet");
    }
    if (static_cast<size_t>(bytes_sent) != request_packet.size()) {
        throw std::runtime_error("receive_device_state: could not send all bytes of packet");
    }

    auto packet = this->receive_packet(timeout);
    auto unpacked = mobility::packets::unpack_packet(packet);

    if (unpacked.header.command_id == mobility::packets::Commands::device_state) {
        mobility::packets::DeviceStatePacket state = {
            .header = unpacked.header,
            .state = mobility::packets::DeviceState()
        };
        state.state = mobility::packets::unpack_device_state_payload(unpacked.payload);
        return state;
    } else {
        throw std::runtime_error("receive_device_state: package received is not package");
    }

}

std::vector<uint8_t> IMobilityDevice::receive_packet(std::chrono::milliseconds timeout) {
    using namespace mobility::packets;

    static std::vector<uint8_t> buffer;  // persistent buffer for streaming input
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Check timeout
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            throw std::runtime_error("Timeout while waiting for packet");
        }

        // Read new data
        uint8_t temp[64];
        int bytes_read = this->read_bytes(temp, sizeof(temp), timeout);
        if (bytes_read < 0)
            throw std::runtime_error("Error reading from device");
        else if (bytes_read == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // Append to buffer
        buffer.insert(buffer.end(), temp, temp + bytes_read);

        // Try parsing while we have at least a header
        while (buffer.size() >= mobility_packet_header_size()) {
            // Search for valid device ID at start of header
            uint32_t dev_id =
                (uint32_t) buffer[0] |
                ((uint32_t) buffer[1] << 8) |
                ((uint32_t) buffer[2] << 16) |
                ((uint32_t) buffer[3] << 24);

            if (dev_id != MOBILITY_DEVICE_ID) {
                // Not aligned — drop one byte and try again
                buffer.erase(buffer.begin());
                continue;
            }

            // Extract header bytes safely
            std::vector<uint8_t> header_buf(buffer.begin(),
                buffer.begin() + mobility_packet_header_size());

            Header hdr;
            try {
                hdr = unpack_header(header_buf);
            } catch (...) {
                // Header corrupted — resync
                buffer.erase(buffer.begin());
                continue;
            }

            size_t full_packet_size = mobility_packet_header_size() + hdr.content_size;
            if (full_packet_size > 1024) {
                // Invalid (too large)
                buffer.erase(buffer.begin());
                continue;
            }

            // Not enough bytes for full packet yet
            if (buffer.size() < full_packet_size)
                break;

            // Verify CRC
            /*uint32_t crc_calc = mobility_crc32(buffer.data(), full_packet_size - sizeof(uint32_t));
            if (crc_calc != hdr.crc32) {
                // Bad CRC — discard first byte, resync
                buffer.erase(buffer.begin());
                continue;
            }*/

            // Valid packet found
            std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + full_packet_size);
            buffer.erase(buffer.begin(), buffer.begin() + full_packet_size);
            return packet;
        }

        // Prevent runaway memory use
        if (buffer.size() > 2048) {
            buffer.clear();
        }
    }
}

