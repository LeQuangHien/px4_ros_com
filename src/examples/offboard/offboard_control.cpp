/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <bridge_msgs/msg/setpoint.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
        offboard_control_mode_publisher_ =
                this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
        trajectory_setpoint_publisher_ =
                this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
        vehicle_command_publisher_ =
                this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
#else
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
        trajectory_setpoint_publisher_ =
             this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

        parse_parameters();

        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        history_policy_,
                        depth_
                ));

        qos.reliability(reliability_policy_);

        // get common timestamp
        timesync_sub_ =
                this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
                                                                   [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                                                       timestamp_.store(msg->timestamp);
                                                                   });

        // manually enable topic statistics via options
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the topic name (default '/statistics')
        options.topic_stats_options.publish_topic = "/statistics_setpoint";

        auto setpoint_callback = [this](const bridge_msgs::msg::Setpoint::UniquePtr msg) {
            auto time_diff = this->now() - msg->header.stamp;
            std::cout << "\n";
            std::cout << "RECEIVED Trajectory frame id: " << msg->header.frame_id << std::endl;
            std::cout << "Time stamp sec: " << msg->header.stamp.sec << std::endl;
            std::cout << "Time stamp nanosec: " << msg->header.stamp.nanosec << std::endl;
            std::cout << "Time diff sec: " << time_diff.seconds() << std::endl;
            std::cout << "Time diff nanosec: " << time_diff.nanoseconds() << std::endl;
            std::cout << "x: " << msg->x << std::endl;
            std::cout << "y: " << msg->y << std::endl;
            std::cout << "z: " << msg->z << std::endl;

            TrajectorySetpoint setpoint{};
            setpoint.timestamp = timestamp_.load();
            setpoint.x = msg->x;
            setpoint.y = msg->y;
            setpoint.z = msg->z;
            setpoint.yaw = -3.14; // [-PI:PI]

            trajectory_setpoint_publisher_->publish(setpoint);
        };

        setpoints_sub_ = this->create_subscription<bridge_msgs::msg::Setpoint>(
                "Setpoint_PubSubTopic", qos,
                setpoint_callback, options);


        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() const;

    void disarm() const;


private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<bridge_msgs::msg::Setpoint>::SharedPtr setpoints_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    size_t depth_;
    rmw_qos_reliability_policy_t reliability_policy_;
    rmw_qos_history_policy_t history_policy_;
    std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
            {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
            {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
    };

    std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
            {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
            {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
    };

    void publish_offboard_control_mode() const;

    void publish_trajectory_setpoint() const;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0) const;

    void parse_parameters() {
        // Parse 'reliability' parameter
        rcl_interfaces::msg::ParameterDescriptor reliability_desc;
        reliability_desc.description = "Reliability QoS setting for the listener";
        reliability_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_reliability_policy_map) {
            reliability_desc.additional_constraints += entry.first + " ";
        }
        const std::string reliability_param = this->declare_parameter(
                "reliability", "reliable", reliability_desc);
        auto reliability = name_to_reliability_policy_map.find(reliability_param);
        if (reliability == name_to_reliability_policy_map.end()) {
            std::ostringstream oss;
            oss << "Invalid QoS reliability setting '" << reliability_param << "'";
            throw std::runtime_error(oss.str());
        }
        reliability_policy_ = reliability->second;

        // Parse 'history' parameter
        rcl_interfaces::msg::ParameterDescriptor history_desc;
        history_desc.description = "History QoS setting for the listener";
        history_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_history_policy_map) {
            history_desc.additional_constraints += entry.first + " ";
        }
        const std::string history_param = this->declare_parameter(
                "history", name_to_history_policy_map.begin()->first, history_desc);
        auto history = name_to_history_policy_map.find(history_param);
        if (history == name_to_history_policy_map.end()) {
            std::ostringstream oss;
            oss << "Invalid QoS history setting '" << history_param << "'";
            throw std::runtime_error(oss.str());
        }
        history_policy_ = history->second;

        // Declare and get remaining parameters
        depth_ = this->declare_parameter("depth", 10);
    }

};


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
    TrajectorySetpoint msg{};
    msg.timestamp = timestamp_.load();
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = -5.0;
    msg.yaw = -3.14; // [-PI:PI]

    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
                                              float param2) const {
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}



int main(int argc, char *argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}
