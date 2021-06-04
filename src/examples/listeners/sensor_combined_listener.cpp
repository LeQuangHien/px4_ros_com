/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
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
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <bridge_msgs/msg/sensor.hpp>


/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node {
public:
    explicit SensorCombinedListener() : Node("sensor_combined_listener") {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);
        //qos.best_effort();

        sensor_publisher_ = this->create_publisher<bridge_msgs::msg::Sensor>("SensorBridge_PubSubTopic",
                                                                             qos);
        subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
                "SensorCombined_PubSubTopic",
#ifdef ROS_DEFAULT_API
                10,
#endif
                [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
                    std::cout << "\n";
                    std::cout << "RECEIVED SENSOR COMBINED DATA" << std::endl;
                    std::cout << "=============================" << std::endl;
                    std::cout << "ts: " << msg->timestamp << std::endl;
                    std::cout << "gyro_rad[0]: " << msg->gyro_rad[0] << std::endl;
                    std::cout << "gyro_rad[1]: " << msg->gyro_rad[1] << std::endl;
                    std::cout << "gyro_rad[2]: " << msg->gyro_rad[2] << std::endl;
                    std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
                    std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative
                              << std::endl;
                    std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
                    std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
                    std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
                    std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;

                    auto sensor = bridge_msgs::msg::Sensor();
                    sensor.header.stamp = this->now();
                    count_++;
                    sensor.header.frame_id = std::to_string(count_);
                    sensor.gyro_rad = msg->gyro_rad;
                    sensor.gyro_integral_dt = msg->gyro_integral_dt;
                    sensor.accelerometer_timestamp_relative = msg->accelerometer_timestamp_relative;
                    sensor.accelerometer_m_s2 = msg->accelerometer_m_s2;
                    sensor.accelerometer_integral_dt = msg->accelerometer_integral_dt;


                    RCLCPP_INFO(this->get_logger(),
                                "Publishing sensor %d: second: %llu nanosecond: %llu gyro_rad[0]: %f gyro_rad[1]: %f gyro_rad[2]: %f ",
                                count_, sensor.header.stamp.sec, sensor.header.stamp.nanosec,
                                sensor.gyro_rad[0],
                                sensor.gyro_rad[1], sensor.gyro_rad[2]);
                    this->sensor_publisher_->publish(sensor);
                });
    }

private:
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
    rclcpp::Publisher<bridge_msgs::msg::Sensor>::SharedPtr sensor_publisher_;
    size_t count_;
};


int main(int argc, char *argv[]) {
    std::cout << "Starting sensor_combined listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorCombinedListener>());

    rclcpp::shutdown();
    return 0;
}
