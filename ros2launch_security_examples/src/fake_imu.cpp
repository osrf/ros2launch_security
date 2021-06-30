// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/trigger.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace ros2launch_security_examples
{

class FakeImu : public rclcpp::Node
{
public:
  explicit FakeImu(const rclcpp::NodeOptions & options)
  : Node("fake_imu", options),
    thread_(
      [this]() {
        auto pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        sensor_msgs::msg::Imu msg;
        // Indicate that only the angular_velocity is available by using -1 for the others.
        msg.orientation_covariance[0] = -1;
        msg.linear_acceleration_covariance[0] = -1;
        // Give bogus covariance values on the diagonal.
        msg.angular_velocity_covariance = {
          0.1, 0.0, 0.0,
          0.0, 0.1, 0.0,
          0.0, 0.0, 0.1
        };
        // Publish Imu messages on a 10Hz loop.
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && this->running_) {
          rate.sleep();
          loop_count_++;
          if (loop_count_ < 10) {
            // Simulate startup time for the IMU.
            continue;
          }
          msg.header.stamp = this->now();
          // Simulate ~1 rad/sec yaw rotation with sine wave noise.
          msg.angular_velocity.z = 1.0 + (0.1 * sin(loop_count_ * 0.1));
          pub->publish(msg);
        }
      })
  {
    reset_imu_service_ = this->create_service<example_interfaces::srv::Trigger>(
      "reset_imu",
      [this](
        const std::shared_ptr<example_interfaces::srv::Trigger::Request>/* request */,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Reseting IMU due to service request.");
        this->loop_count_ = 0;
        response->success = true;
      }
    );
  }

  virtual ~FakeImu()
  {
    this->running_ = false;
    thread_.join();
  }

private:
  bool running_{true};
  size_t loop_count_{0};
  std::thread thread_;
  std::shared_ptr<rclcpp::Service<example_interfaces::srv::Trigger>> reset_imu_service_;
};

}  // namespace ros2launch_security_examples

RCLCPP_COMPONENTS_REGISTER_NODE(ros2launch_security_examples::FakeImu)
