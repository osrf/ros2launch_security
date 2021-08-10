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
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/trigger.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace ros2launch_security_examples
{

class ImuSink : public rclcpp::Node
{
public:
  explicit ImuSink(const rclcpp::NodeOptions & options)
  : Node("imu_sink", options)
  {
    reset_imu_client_ = this->create_client<example_interfaces::srv::Trigger>("reset_imu");
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu",
      10,
      [this](const sensor_msgs::msg::Imu & msg) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Recieved imu with yaw rate = " << msg.angular_velocity.z);
        number_of_imu_messages_++;
        if (number_of_imu_messages_ > 10) {
          // Every N messages, reset the IMU using the service.
          this->reset_imu();
        }
      }
    );
  }

  void
  reset_imu()
  {
    RCLCPP_INFO(this->get_logger(), "Reseting IMU via service call...");
    this->number_of_imu_messages_ = 0;
    auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
    this->reset_imu_client_->async_send_request(
      request,
      [this](rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture shared_future) {
        auto response = shared_future.get();
        if (!response->success) {
          RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Failed to reset IMU: " << response->message);
        }
      }
    );
  }

private:
  size_t number_of_imu_messages_{0};
  std::shared_ptr<rclcpp::Client<example_interfaces::srv::Trigger>> reset_imu_client_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> subscription_;
};

}  // namespace ros2launch_security_examples

RCLCPP_COMPONENTS_REGISTER_NODE(ros2launch_security_examples::ImuSink)
