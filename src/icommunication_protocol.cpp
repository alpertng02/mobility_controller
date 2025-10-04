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

#include "icommunication_protocol.hpp"
#include <vector>

ICommunicationProtocol::~ICommunicationProtocol() {}

bool ICommunicationProtocol::send_command_packet(MobilityCommandPacket& packet) {
    packet.frame_start = FRAME_START;
    packet.frame_end = FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

bool ICommunicationProtocol::send_init_packet(MobilityInitPacket& packet) {
    packet.frame_start = FRAME_START;
    packet.frame_end = FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

static std::vector<uint8_t> buffer{};
bool ICommunicationProtocol::receive_motor_feedback(MobilityFeedbackPacket* feedback, std::chrono::milliseconds timeout) {
    if (!feedback) return false;

    uint8_t temp[64]; // read in small chunks
    int bytes_read = read_bytes(temp, sizeof(temp), timeout);

    if (bytes_read <= 0) return false;

    // Append new data to sliding buffer
    buffer.insert(buffer.end(), temp, temp + bytes_read);

    while (buffer.size() >= sizeof(MobilityFeedbackPacket)) {
        // Check for FRAME_START at current position
        uint32_t frame_start = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
        if (frame_start == FRAME_START) {
            // Possible start of packet, check if we have enough bytes
            if (buffer.size() < sizeof(MobilityFeedbackPacket)) break;

            MobilityFeedbackPacket* pkt = reinterpret_cast<MobilityFeedbackPacket*>(buffer.data());
            if (pkt->frame_end == FRAME_END) {
                // Valid packet, copy to feedback
                *feedback = *pkt;
                // Remove consumed bytes
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(MobilityFeedbackPacket));
                return true;
            } else {
                // FRAME_END mismatch, discard first byte and resync
                buffer.erase(buffer.begin());
            }
        } else {
            // FRAME_START not found, discard first byte
            buffer.erase(buffer.begin());
        }
    }
    return false;
}

bool ICommunicationProtocol::receive_state_feedback(MobilityStatePacket* packet, std::chrono::milliseconds timeout) {
    if (!packet) return false;

    uint8_t temp[64]; // read in small chunks
    int bytes_read = read_bytes(temp, sizeof(temp), timeout);

    if (bytes_read <= 0) return false;

    // Append new data to sliding buffer
    buffer.insert(buffer.end(), temp, temp + bytes_read);

    while (buffer.size() >= sizeof(MobilityStatePacket)) {
        // Check for FRAME_START at current position
        uint32_t frame_start = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
        if (frame_start == FRAME_START) {
            // Possible start of packet, check if we have enough bytes
            if (buffer.size() < sizeof(MobilityStatePacket)) break;

            MobilityStatePacket* pkt = reinterpret_cast<MobilityStatePacket*>(buffer.data());
            if (pkt->frame_end == FRAME_END) {
                // Valid packet, copy to feedback
                *packet = *pkt;
                // Remove consumed bytes
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(MobilityStatePacket));
                return true;
            } else {
                // FRAME_END mismatch, discard first byte and resync
                buffer.erase(buffer.begin());
            }
        } else {
            // FRAME_START not found, discard first byte
            buffer.erase(buffer.begin());
        }
    }
    return false;
}


// Send target wheel velocities
bool ICommunicationProtocol::send_target_velocity(const std::array<float, 4>& target_velocities) {
    MobilityCommandPacket pkt {};

    pkt.command_id = MobilityCommands::set_velocity;
    for (size_t i = 0; i < target_velocities.size(); i++) {
        pkt.values[i] = target_velocities[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send PWM duty cycles
bool ICommunicationProtocol::send_pwm_duty(const std::array<float, 4>& duties) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_dutycycle;
    for (size_t i = 0; i < duties.size(); i++) {
        pkt.values[i] = duties[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send maximum PWM duty
bool ICommunicationProtocol::send_max_pwm_duty(float max_pwm_duty) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_maximum_dutycycle;
    pkt.values[0] = max_pwm_duty;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send velocity filter cutoff
bool ICommunicationProtocol::send_lowpass_cutoff(float fc) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_lowpass_fc;
    pkt.values[0] = fc;

    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send PID parameters
bool ICommunicationProtocol::send_pid_params(float kp, float ki, float kd) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_pid_params;
    pkt.values[0] = kp;
    pkt.values[1] = ki;
    pkt.values[2] = kd;

    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send PID bounds
bool ICommunicationProtocol::send_pid_bounds(float p_bound, float i_bound, float d_bound) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_pid_bounds;
    pkt.values[0] = p_bound;
    pkt.values[1] = i_bound;
    pkt.values[2] = d_bound;

    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send feedforward parameters
bool ICommunicationProtocol::send_feedforward_params(float kv, float voff) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_ff_params;
    pkt.values[0] = kv;
    pkt.values[1] = 0.0f; // ka
    pkt.values[2] = 0.0f; // kj
    pkt.values[3] = voff;

    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send maximum velocity
bool ICommunicationProtocol::send_max_velocity(float max_vel) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_max_velocity;
    pkt.values[0] = max_vel;
    send_command_packet(pkt);
    return true;
}

// Send feedback rate
bool ICommunicationProtocol::send_feedback_rate(float feedback_rate_hz) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_feedback_hz;
    pkt.values[0] = feedback_rate_hz;

    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Send motor control rate
bool ICommunicationProtocol::send_motor_control_rate(float motor_control_rate_hz) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::set_control_hz;
    pkt.values[0] = motor_control_rate_hz;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Open loop enable / disable
bool ICommunicationProtocol::send_open_loop_enable(bool enable) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::open_loop_enable;
    pkt.values[0] = enable ? 1.0f : 0.0f;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Start / Stop motor control
bool ICommunicationProtocol::send_running_mode_enable(bool enable) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::running_mode_enable;
    pkt.values[0] = enable ? 1.0f : 0.0f;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Enable / disable init mode
bool ICommunicationProtocol::send_init_mode_enable(bool enable) {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::init_mode_enable;
    pkt.values[0] = enable ? 1.0f : 0.0f;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Enable / disable init mode
bool ICommunicationProtocol::request_device_state() {
    MobilityCommandPacket pkt {};
    pkt.command_id = MobilityCommands::send_device_state;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}