/****************************************************************************
 *
 * Copyright 2019 PX4 Development Team. All rights reserved.
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
 * @brief Vehicle GPS position uORB topic listener example
 * @file vehicle_global_position_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <bridge_msgs/msg/vehicle_position.hpp>


/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node {
public:
    explicit VehicleGpsPositionListener() : Node("vehicle_global_position_listener") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);
        //qos.best_effort();

        position_publisher_ = this->create_publisher<bridge_msgs::msg::VehiclePosition>("VehiclePosition_PubSubTopic",
                                                                                        qos);
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "VehicleGpsPosition_PubSubTopic",
#ifdef ROS_DEFAULT_API
                10,
#endif
                [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
                    std::cout << "\n\n\n\n\n\n\n\n\n\n";
                    std::cout << "RECEIVED VEHICLE LOCAl POSITION DATA" << std::endl;
                    std::cout << "==================================" << std::endl;
                    std::cout << "ts: " << msg->timestamp << std::endl;
                    std::cout << "lat: " << msg->x << std::endl;
                    std::cout << "lon: " << msg->y << std::endl;
                    std::cout << "alt: " << msg->z << std::endl;

                    auto vehicle_position = bridge_msgs::msg::VehiclePosition();
                    vehicle_position.header.stamp = this->now();
                    count_++;
                    vehicle_position.header.frame_id = std::to_string(count_);
                    vehicle_position.x = msg->x;
                    vehicle_position.y = msg->y;
                    vehicle_position.z = msg->z;

                    RCLCPP_INFO(this->get_logger(),
                                "Publishing gps location %d: second: %llu nanosecond: %llu x: %f y: %f z: %f ",
                                count_, vehicle_position.header.stamp.sec, vehicle_position.header.stamp.nanosec, vehicle_position.x,
                                vehicle_position.y, vehicle_position.z);
                    this->position_publisher_->publish(vehicle_position);
                });
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
    rclcpp::Publisher<bridge_msgs::msg::VehiclePosition>::SharedPtr position_publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    std::cout << "Starting vehicle_global_position listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleGpsPositionListener>());

    rclcpp::shutdown();
    return 0;
}
