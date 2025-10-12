/**
 *@file mobility_control_node_usb.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-10-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "usb_device.hpp"

class MobilityControlNode : public rclcpp::Node {

public:
    MobilityControlNode() : Node("mobility_control_node") {

        init_parameters();

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&MobilityControlNode::cmd_vel_callback, this, std::placeholders::_1));

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_,
            rclcpp::SystemDefaultsQoS());

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_,
            rclcpp::SystemDefaultsQoS());

        size_t jointCount = wheel_joint_names_.size();
        wheel_joint_states_.name.reserve(jointCount);
        wheel_joint_states_.position.reserve(jointCount);
        wheel_joint_states_.velocity.reserve(jointCount);
        wheel_joint_states_.name.resize(jointCount);
        wheel_joint_states_.position.resize(jointCount);
        wheel_joint_states_.velocity.resize(jointCount);
        for (size_t i = 0; i < wheel_joint_names_.size(); i++) {
            wheel_joint_states_.name[i] = wheel_joint_names_[i];
        }

        // Initialize timers but keep them disabled
        odom_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 / feedback_rate_hz_),
            std::bind(&MobilityControlNode::publish_odometry_and_joint_states, this));
        odom_timer_->cancel();
        command_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 / command_publish_rate_hz_),
            std::bind(&MobilityControlNode::command_controller, this));
        command_timer_->cancel();

        if (protocol_type_ == "usb") {
            device_ = std::make_unique<UsbMobilityDevice>();
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unsupported protocol: %s", protocol_type_.c_str());
            throw std::runtime_error("Unsupported protocol");
        }
        device_connection_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 * reconnection_retry_period_sec_),
            std::bind(&MobilityControlNode::device_connection_callback, this));

    }
private:

    enum WheelIndex {
        front_left = 0,
        front_right = 1,
        back_left = 2,
        back_right = 3
    };

    std::unique_ptr<IMobilityDevice> device_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_ {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_ {};
    rclcpp::TimerBase::SharedPtr odom_timer_ {};
    rclcpp::TimerBase::SharedPtr command_timer_ {};
    rclcpp::TimerBase::SharedPtr device_connection_timer_ {};
    sensor_msgs::msg::JointState wheel_joint_states_ {};

    std::string protocol_type_ { "usb" };

    std::string cmd_vel_topic_ { "cmd_vel" };
    std::string odom_topic_ { "odom" };
    std::string joint_states_topic_ { "joint_states" };

    bool overwrite_pinout_ { false };
    std::vector<int64_t> motor_lpwm_pins_ { 2, 6, 4, 8 };
    std::vector<int64_t> motor_rpwm_pins_ { 3, 7, 5, 9 };
    std::vector<int64_t> motor_encoder_a_pins_ { 10, 14, 12, 2 };
    std::vector<int64_t> motor_swap_dirs_ { 0, 0, 0, 0 };

    double wheel_separation_ = 1.0;
    double wheel_radius_ = 0.1;
    double wheel_reduction_ = 100;
    double wheel_encoder_cpr_ = 64;
    std::vector<double> wheel_encoder_velocities_steps_sec {
        0.0, 0.0, 0.0, 0.0
    };

    std::vector<std::string> wheel_joint_names_ {
        "front_left_wheel",
        "front_right_wheel",
        "back_left_wheel",
        "back_right_wheel"
    };
    std::string base_frame_id_ { "base_link" };
    std::string odom_frame_id_ { "odom" };

    std::vector<double> pose_covariance_diagonal_ { 0.002, 0.002, 0.0, 0.0, 0.0, 0.02 };
    std::vector<double> twist_covariance_diagonal_ { 0.002, 0.0, 0.0, 0.0, 0.0, 0.02 };

    double max_pwm_dutycycle_ { 60.0 };
    double max_velocity_ { 30000.0 };
    double velocity_filter_cutoff_hz_ { 100.0 };

    double pid_kp_ { 1.0 };
    double pid_ki_ { 0.01 };
    double pid_kd_ { 0.0001 };

    double pid_p_bound_ { 100.0f };
    double pid_i_bound_ { 50.0f };
    double pid_d_bound_ { 50.0f };

    double motor_control_rate_hz_ = 1000.0;
    double feedback_rate_hz_ = 100.0;
    double command_publish_rate_hz_ = 100.0;

    double reconnection_retry_period_sec_ { 1.0 };

    double cmd_vel_timeout_sec_ = 0.5;

    bool is_open_loop_ = true;
    bool encoders_connected_ = false;

    bool linear_x_has_velocity_limits_ = false;
    double linear_x_max_velocity_ = 1.0;
    double linear_x_min_velocity_ = -1.0;

    bool angular_z_has_velocity_limits_ = false;
    double angular_z_max_velocity_ = 1.0;
    double angular_z_min_velocity_ = -1.0;

    bool linear_x_has_acceleration_limits_ = false;
    double linear_x_max_acceleration_ = 5.0;
    double linear_x_min_acceleration_ = -5.0;

    bool angular_z_has_acceleration_limits_ = false;
    double angular_z_max_acceleration_ = 5.0;
    double angular_z_min_acceleration_ = -5.0;

    std::array<float, 4> target_velocities_ { 0, 0, 0, 0 };

    rclcpp::Time prev_cmd_vel_time_ { this->get_clock()->now() };
    rclcpp::Time prev_odom_time_ { this->get_clock()->now() };
    rclcpp::Time prev_log_time_ { this->get_clock()->now() };
    rclcpp::Time prev_command_time_ { this->get_clock()->now() };

    geometry_msgs::msg::Twist cmd_vel_received_ {};
    geometry_msgs::msg::Twist target_cmd_vel_ {};
    geometry_msgs::msg::Twist prev_target_cmd_vel_ {};

    double heading_ {};
    double pos_x_ {};
    double pos_y_ {};

    mobility::packets::Feedback feedback_ {};


    void enable_timers(bool enable) {
        if (enable) {
            if (odom_timer_->is_canceled()) {
                RCLCPP_INFO(this->get_logger(), "Enabling odometry and control timers.");
                odom_timer_->reset();
                command_timer_->reset();
            }
        } else {
            if (!odom_timer_->is_canceled()) {
                RCLCPP_WARN(this->get_logger(), "Disabling odometry and control timers.");
                odom_timer_->cancel();
                command_timer_->cancel();
            }
        }
    }

    bool try_connect_device(std::chrono::milliseconds timeout) {

        auto available_ports = device_->list_all_ports();

        if (available_ports.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "No ports available");
            return false;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Available ports:");
        for (const auto& port : available_ports) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "\t%s\n", port.c_str());
        }

        for (const auto& port : available_ports) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Trying to connect to port: %s", port.c_str());
            if (!device_->open(port)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Port %s cannot open!", port.c_str());
                continue;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Connected to port: %s", port.c_str());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Trying to initialize device...");
            if (device_->init_device(this->get_init_packet(), timeout)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Device is initialized!");
                return true;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not initialize device...");
                device_->close();
            }
        }
        return false;
    }

    void device_connection_callback() {
        if (!device_->is_open()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Device Disconnected");
            enable_timers(false);
            if (try_connect_device(std::chrono::milliseconds(200))) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Device Connected");
                prev_odom_time_ = this->get_clock()->now();
                enable_timers(true);
            }
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
        prev_cmd_vel_time_ = this->get_clock()->now();

        cmd_vel_received_ = msg;

        if (linear_x_has_velocity_limits_) {
            cmd_vel_received_.linear.x = std::clamp(cmd_vel_received_.linear.x, linear_x_min_velocity_, linear_x_max_velocity_);
        }
        if (angular_z_has_velocity_limits_) {
            cmd_vel_received_.angular.z = std::clamp(cmd_vel_received_.angular.z, angular_z_min_velocity_, angular_z_max_velocity_);
        }
    }


    void publish_odometry_and_joint_states() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - prev_odom_time_).seconds();

        try {
            feedback_ = device_->receive_motor_feedback(std::chrono::milliseconds(20)).feedback;

            for (int i = 0; i < 4; i++) {
                wheel_encoder_velocities_steps_sec[i] = feedback_.velocities[i];
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "PWM: D0:%.2f, D1:%.2f, D2:%.2f, D3:%.2f",
                feedback_.pwm_duties[0], feedback_.pwm_duties[1], feedback_.pwm_duties[2], feedback_.pwm_duties[3]);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Pos: P0:%.2f, P1:%.2f, P2:%.2f, P3:%.2f",
                feedback_.positions[0], feedback_.positions[1], feedback_.positions[2], feedback_.positions[3]);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Vel: V0:%.2f, V1:%.2f, V2:%.2f, V3:%.2f",
                feedback_.velocities[0], feedback_.velocities[1], feedback_.velocities[2], feedback_.velocities[3]);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "TargetVel: T0:%.2f, T1:%.2f, T2:%.2f, T3:%.2f",
                target_velocities_[0], target_velocities_[1], target_velocities_[2], target_velocities_[3]);
            prev_log_time_ = current_time;

            
            double robot_linear_velocity = 0;
            double robot_angular_velocity = 0;
            double left_side_velocity = 0;
            double right_side_velocity = 0;
            std::vector<double>wheel_angular_velocities { 0.0, 0.0, 0.0, 0.0 };
            
            if (!encoders_connected_) {
                
                robot_linear_velocity = target_cmd_vel_.linear.x;
                robot_angular_velocity = target_cmd_vel_.angular.z;
                
                left_side_velocity = robot_linear_velocity - (wheel_separation_ / 2.0) * robot_angular_velocity;
                right_side_velocity = robot_linear_velocity + (wheel_separation_ / 2.0) * robot_angular_velocity;
                
                wheel_angular_velocities[front_left] = left_side_velocity / wheel_radius_;
                wheel_angular_velocities[back_left] = left_side_velocity / wheel_radius_;
                wheel_angular_velocities[front_right] = right_side_velocity / wheel_radius_;
                wheel_angular_velocities[back_right] = right_side_velocity / wheel_radius_;
                
            } else {
                for (size_t i = 0; i < wheel_encoder_velocities_steps_sec.size(); i++) {
                    double motor_rotations_per_sec = wheel_encoder_velocities_steps_sec[i] / (wheel_encoder_cpr_ * wheel_reduction_);
                    wheel_angular_velocities[i] = motor_rotations_per_sec * 2.0 * M_PI;
                }
                
                left_side_velocity =
                (wheel_angular_velocities[front_left] + wheel_angular_velocities[back_left]) / 2.0;
                right_side_velocity =
                (wheel_angular_velocities[front_right] + wheel_angular_velocities[back_right]) / 2.0;
                
                robot_linear_velocity = (left_side_velocity + right_side_velocity) / 2.0;
                robot_angular_velocity = (right_side_velocity - left_side_velocity) / wheel_separation_;
            }
            
            double delta_heading = robot_angular_velocity * dt;
            double delta_x = (robot_linear_velocity * cos(heading_)) * dt;
            double delta_y = (robot_linear_velocity * sin(heading_)) * dt;
            
            pos_x_ += delta_x;
            pos_y_ += delta_y;
            heading_ += delta_heading;
            
            // https://github.com/linorobot/linorobot/blob/master/src/lino_base.cpp
            
            tf2::Quaternion odomQuat {};
            odomQuat.setRPY(0, 0, heading_);
            
            nav_msgs::msg::Odometry odom_msg {};
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = odom_frame_id_;
            odom_msg.child_frame_id = base_frame_id_;
            
            odom_msg.pose.pose.position.x = pos_x_;
            odom_msg.pose.pose.position.y = pos_y_;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.x = odomQuat.x();
            odom_msg.pose.pose.orientation.y = odomQuat.y();
            odom_msg.pose.pose.orientation.z = odomQuat.z();
            odom_msg.pose.pose.orientation.w = odomQuat.w();
            
            
            odom_msg.twist.twist.linear.x = robot_linear_velocity;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = robot_angular_velocity;
            
            for (size_t i = 0; i < pose_covariance_diagonal_.size(); i++) {
                if (i * 7 < odom_msg.pose.covariance.size()) {
                    odom_msg.pose.covariance[i * 7] = pose_covariance_diagonal_[i];
                    odom_msg.twist.covariance[i * 7] = twist_covariance_diagonal_[i];
                }
            }
            odom_publisher_->publish(odom_msg);
            
            
            wheel_joint_states_.header.frame_id = base_frame_id_;
            wheel_joint_states_.header.stamp = current_time;
            for (size_t i = 0; i < wheel_joint_names_.size(); i++) {
                wheel_joint_states_.velocity[i] = wheel_angular_velocities[i];
                if (is_open_loop_) {
                    wheel_joint_states_.position[i] += wheel_joint_states_.velocity[i] * dt;
                } else {
                    wheel_joint_states_.position[i] = feedback_.positions[i] * 2.0 * M_PI / (wheel_encoder_cpr_ * wheel_reduction_);
                }
            }
            joint_state_publisher_->publish(wheel_joint_states_);
            
            prev_odom_time_ = current_time;

        } catch (std::runtime_error& err) {
            RCLCPP_WARN(this->get_logger(), "Could not receive joint feedbacks");
        }
    }
    
    void command_controller() {
        
        auto current_time = this->get_clock()->now();
        double dt = (current_time - prev_command_time_).seconds();
        prev_command_time_ = current_time;

        std::array<float, mobility::packets::k_motor_count> wheel_velocities {};

        // Copy the latest received cmd_vel

        target_cmd_vel_ = cmd_vel_received_;

        if ((current_time - prev_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_) {
            target_cmd_vel_.linear.x = 0.0;
            target_cmd_vel_.linear.y = 0.0;
            target_cmd_vel_.linear.z = 0.0;
            target_cmd_vel_.angular.x = 0.0;
            target_cmd_vel_.angular.y = 0.0;
            target_cmd_vel_.angular.z = 0.0;
        }

        // --- Linear X acceleration limiting ---
        if (linear_x_has_acceleration_limits_) {
            double delta_v = target_cmd_vel_.linear.x - prev_target_cmd_vel_.linear.x;
            double max_increase = linear_x_max_acceleration_ * dt;
            double max_decrease = linear_x_min_acceleration_ * dt;
            delta_v = std::clamp(delta_v, max_decrease, max_increase);
            target_cmd_vel_.linear.x = prev_target_cmd_vel_.linear.x + delta_v;
        }

        // --- Angular Z acceleration limiting ---
        if (angular_z_has_acceleration_limits_) {
            double delta_v = target_cmd_vel_.angular.z - prev_target_cmd_vel_.angular.z;
            double max_increase = angular_z_max_acceleration_ * dt;
            double max_decrease = angular_z_min_acceleration_ * dt;
            delta_v = std::clamp(delta_v, max_decrease, max_increase);
            target_cmd_vel_.angular.z = prev_target_cmd_vel_.angular.z + delta_v;
        }

        // --- Velocity limits (final clamp) ---
        if (linear_x_has_velocity_limits_) {
            target_cmd_vel_.linear.x = std::clamp(target_cmd_vel_.linear.x, linear_x_min_velocity_, linear_x_max_velocity_);
        }
        if (angular_z_has_velocity_limits_) {
            target_cmd_vel_.angular.z = std::clamp(target_cmd_vel_.angular.z, angular_z_min_velocity_, angular_z_max_velocity_);
        }

        // Update stored velocity
        prev_target_cmd_vel_ = target_cmd_vel_;

        // --- Use filtered velocity values for control ---
        double v = target_cmd_vel_.linear.x;
        double w = target_cmd_vel_.angular.z;

        // Differential drive kinematics
        double left_vel = v - (wheel_separation_ / 2.0) * w;
        double right_vel = v + (wheel_separation_ / 2.0) * w;

        bool res = false;

        if (is_open_loop_) {
            double max_velocity = std::max(std::abs(linear_x_max_velocity_), 1e-6); // avoid division by zero

            wheel_velocities[front_left] = std::clamp((left_vel / max_velocity) * 100.0, -max_pwm_dutycycle_, max_pwm_dutycycle_);
            wheel_velocities[back_left] = wheel_velocities[front_left];
            wheel_velocities[front_right] = std::clamp((right_vel / max_velocity) * 100.0, -max_pwm_dutycycle_, max_pwm_dutycycle_);
            wheel_velocities[back_right] = wheel_velocities[front_right];

            res = device_->send_motor_dutycycles(wheel_velocities);
        } else {
            wheel_velocities[front_left] = left_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            wheel_velocities[back_left] = wheel_velocities[front_left];
            wheel_velocities[front_right] = right_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            wheel_velocities[back_right] = wheel_velocities[front_right];

            target_velocities_ = wheel_velocities;
            res = device_->send_motor_velocities(wheel_velocities);
        }

        if (!res) {
            enable_timers(false);
        }
    }


    mobility::packets::InitPacket get_init_packet() {
        mobility::packets::InitPacket pkt {};

        pkt.overwrite_pinout = overwrite_pinout_ ? 1 : 0;
        for (int i = 0; i < 4; i++) {
            pkt.lpwm_pins[i] = motor_lpwm_pins_[i];
            pkt.rpwm_pins[i] = motor_rpwm_pins_[i];
            pkt.encoder_a_pins[i] = motor_encoder_a_pins_[i];
            pkt.motor_swap_dirs[i] = motor_swap_dirs_[i];
        }
        pkt.max_dutycycle = max_pwm_dutycycle_;
        pkt.max_velocity = max_velocity_;
        pkt.lowpass_fc = velocity_filter_cutoff_hz_;

        pkt.kp = pid_kp_;
        pkt.ki = pid_ki_;
        pkt.kd = pid_kd_;

        pkt.p_bound = pid_p_bound_;
        pkt.i_bound = pid_i_bound_;
        pkt.d_bound = pid_d_bound_;

        pkt.feedback_hz = feedback_rate_hz_;
        pkt.control_hz = motor_control_rate_hz_;

        pkt.is_open_loop = is_open_loop_ ? 1 : 0;

        return pkt;
    }


    void init_parameters() {

        protocol_type_ = this->declare_parameter("protocol", protocol_type_);

        command_publish_rate_hz_ = this->declare_parameter("command_publish_rate", command_publish_rate_hz_);
        feedback_rate_hz_ = this->declare_parameter("feedback_rate", feedback_rate_hz_);
        motor_control_rate_hz_ = this->declare_parameter("motor_control_rate", motor_control_rate_hz_);

        cmd_vel_topic_ = this->declare_parameter("cmd_vel_topic", cmd_vel_topic_);
        odom_topic_ = this->declare_parameter("odom_topic", odom_topic_);
        joint_states_topic_ = this->declare_parameter("joint_states_topic", joint_states_topic_);

        wheel_joint_names_ = this->declare_parameter("wheel_joint_names", wheel_joint_names_);

        wheel_separation_ = this->declare_parameter("wheel_separation", wheel_separation_);
        wheel_radius_ = this->declare_parameter("wheel_radius", wheel_radius_);
        wheel_reduction_ = this->declare_parameter("wheel_reduction", wheel_reduction_);
        wheel_encoder_cpr_ = this->declare_parameter("wheel_encoder_cpr", wheel_encoder_cpr_);

        max_pwm_dutycycle_ = this->declare_parameter("max_pwm_dutycycle", max_pwm_dutycycle_);
        velocity_filter_cutoff_hz_ = this->declare_parameter("velocity_filter_cutoff", velocity_filter_cutoff_hz_);

        pid_kp_ = this->declare_parameter("pid_kp", pid_kp_);
        pid_ki_ = this->declare_parameter("pid_ki", pid_ki_);
        pid_kd_ = this->declare_parameter("pid_kd", pid_kd_);

        pid_p_bound_ = this->declare_parameter("pid_p_bound", pid_p_bound_);
        pid_i_bound_ = this->declare_parameter("pid_i_bound", pid_i_bound_);
        pid_d_bound_ = this->declare_parameter("pid_d_bound", pid_d_bound_);

        is_open_loop_ = this->declare_parameter("is_open_loop", is_open_loop_);
        encoders_connected_ = this->declare_parameter("encoders_connected", encoders_connected_);

        overwrite_pinout_ = this->declare_parameter("overwrite_pinout", overwrite_pinout_);
        motor_lpwm_pins_ = this->declare_parameter("motor_lpwm_pins", motor_lpwm_pins_);
        motor_rpwm_pins_ = this->declare_parameter("motor_rpwm_pins", motor_rpwm_pins_);
        motor_encoder_a_pins_ = this->declare_parameter("motor_encoder_a_pins", motor_encoder_a_pins_);
        motor_swap_dirs_ = this->declare_parameter("motor_swap_dirs", motor_swap_dirs_);

        base_frame_id_ = this->declare_parameter("base_frame_id", base_frame_id_);
        odom_frame_id_ = this->declare_parameter("odom_frame_id", odom_frame_id_);

        cmd_vel_timeout_sec_ = this->declare_parameter("cmd_vel_timeout", cmd_vel_timeout_sec_);

        pose_covariance_diagonal_ = this->declare_parameter("pose_covariance_diagonal", pose_covariance_diagonal_);
        twist_covariance_diagonal_ = this->declare_parameter("twist_covariance_diagonal", twist_covariance_diagonal_);

        linear_x_has_velocity_limits_ = this->declare_parameter("linear.x.has_velocity_limits", linear_x_has_velocity_limits_);
        linear_x_max_velocity_ = this->declare_parameter("linear.x.max_velocity", linear_x_max_velocity_);
        linear_x_min_velocity_ = this->declare_parameter("linear.x.min_velocity", linear_x_min_velocity_);

        linear_x_has_acceleration_limits_ = this->declare_parameter("linear.x.has_acceleration_limits", linear_x_has_acceleration_limits_);
        linear_x_max_acceleration_ = this->declare_parameter("linear.x.max_acceleration", linear_x_max_acceleration_);
        linear_x_min_acceleration_ = this->declare_parameter("linear.x.min_acceleration", linear_x_min_acceleration_);

        angular_z_has_velocity_limits_ = this->declare_parameter("angular.z.has_velocity_limits", angular_z_has_velocity_limits_);
        angular_z_max_velocity_ = this->declare_parameter("angular.z.max_velocity", angular_z_max_velocity_);
        angular_z_min_velocity_ = this->declare_parameter("angular.z.min_velocity", angular_z_min_velocity_);

        angular_z_has_acceleration_limits_ = this->declare_parameter("angular.z.has_acceleration_limits", angular_z_has_acceleration_limits_);
        angular_z_max_acceleration_ = this->declare_parameter("angular.z.max_acceleration", angular_z_max_acceleration_);
        angular_z_min_acceleration_ = this->declare_parameter("angular.z.min_acceleration", angular_z_min_acceleration_);
    }

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MobilityControlNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}