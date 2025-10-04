#ifndef __ICOMMUNICATION_PROTOCOL_H__
#define __ICOMMUNICATION_PROTOCOL_H__

#include <string>
#include <chrono>
#include <array>
#include <vector>

#include <mobility_packets.h>

class ICommunicationProtocol {
public:

    virtual bool open(const std::string& dev) = 0;

    virtual bool is_open() = 0;

    virtual std::vector<std::string> list_all_devices() = 0;

    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;

    virtual bool send_command_packet(MobilityCommandPacket& packet);
    virtual bool send_init_packet(MobilityInitPacket& packet);
    
    virtual bool receive_motor_feedback(MobilityFeedbackPacket* packet, std::chrono::milliseconds timeout);
    virtual bool receive_state_feedback(MobilityStatePacket* packet, std::chrono::milliseconds timeout);
        
    // Send target wheel velocities
    virtual bool send_target_velocity(const std::array<float, 4>& target_velocities);

    // Send PWM duty cycles
    virtual bool send_pwm_duty(const std::array<float, 4>& duties);

    // Send maximum PWM duty
    virtual bool send_max_pwm_duty(float max_pwm_duty);

    // Send velocity filter cutoff
    virtual bool send_lowpass_cutoff(float fc);
    // Send PID parameters
    virtual bool send_pid_params(float kp, float ki, float kd);

    // Send PID bounds
    virtual bool send_pid_bounds(float p_bound, float i_bound, float d_bound);

    // Send feedforward parameters
    virtual bool send_feedforward_params(float kv, float voff);

    // Send maximum velocity
    virtual bool send_max_velocity(float max_vel);

    // Send feedback rate
    virtual bool send_feedback_rate(float feedback_rate_hz);

    // Send motor control rate
    virtual bool send_motor_control_rate(float motor_control_rate_hz);

    // Open loop enable / disable
    virtual bool send_open_loop_enable(bool enable);

    // Start / Stop controller
    virtual bool send_running_mode_enable(bool enable);

    virtual bool send_init_mode_enable(bool enable);

    virtual bool request_device_state();

    virtual void close() = 0;

    virtual ~ICommunicationProtocol() = 0;
};

#endif // __ICOMMUNICATION_PROTOCOL_H__