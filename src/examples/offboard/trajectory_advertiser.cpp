//
// Created by Hien Le (lequanghien247@gmail.com) on 07.05.21.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;


class TrajectoryAdvertiser : public rclcpp::Node
{
public:
    TrajectoryAdvertiser(): Node("trajectory_advertiser") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

//example1.cpp
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);
        qos.best_effort();
      /*  qos_ = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        // The history policy determines how messages are saved until taken by
                        // the reader.
                        // KEEP_ALL saves all messages until they are taken.
                        // KEEP_LAST enforces a limit on the number of messages that are saved,
                        // specified by the "depth" parameter.
                        rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
                        // Depth represents how many messages to store in history when the
                        // history policy is KEEP_LAST.
                        10
                ));
        // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // ensure that every message gets received in order, or best effort, meaning that the transport
        // makes no guarantees about the order or reliability of delivery.
        qos_.reliability(rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);*/

#ifdef ROS_DEFAULT_API
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", qos);
#else
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", qos);
#endif
        auto timer_callback =
                [this]()->void {
                    auto setpoint= px4_msgs::msg::TrajectorySetpoint();
                    setpoint.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
                    std::string name = "test";
                    setpoint.x = count_++;
                    setpoint.y = 0.0;
                    setpoint.z = -3.0;
                    RCLCPP_INFO(this->get_logger(), "Publishing setpoint %d: time: %llu x: %f y: %f z: %f ",
                                count_ - 1, setpoint.timestamp, setpoint.x, setpoint.y, setpoint.z);
                    this->publisher_->publish(setpoint);
                };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
    size_t count_;
    //rclcpp::QoS qos_;
};

int main(int argc, char * argv[])
{
    std::cout << "Starting TrajectorySetpoint advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryAdvertiser>());
    rclcpp::shutdown();
    return 0;
}

