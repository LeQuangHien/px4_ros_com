//
// Created by Hien Le (lequanghien247@gmail.com) on 07.05.21.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;


class TrajectoryAdvertiser : public rclcpp::Node {
public:
    TrajectoryAdvertiser() : Node("trajectory_advertiser") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);
        qos.best_effort();

        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("Trajectory_PubSubTopic", qos);
        auto timer_callback =
                [this]() -> void {
                    auto setpoint = px4_msgs::msg::TrajectorySetpoint();
                    setpoint.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now()).time_since_epoch().count();
                    setpoint.x = count_++;
                    setpoint.y = 0.0;
                    setpoint.z = -3.0;
                    RCLCPP_INFO(this->get_logger(), "Publishing setpoint %d: time: %llu x: %f y: %f z: %f ",
                                count_ - 1, setpoint.timestamp, setpoint.x, setpoint.y, setpoint.z);
                    this->publisher_->publish(setpoint);
                };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    std::cout << "Starting TrajectorySetpoint advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryAdvertiser>());
    rclcpp::shutdown();
    return 0;
}

