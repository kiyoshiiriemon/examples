// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <random>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  std::random_device rd;
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("topic");
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    static int fcnt = 0;
    size_t data_sizes[] = {25, 250, 2500, 25000, 250000};
    if (count_ >= 5) {
        return;
    }
    using builtin_interfaces::msg::Time;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0., 1.);
    auto message = sensor_msgs::msg::LaserScan();
    message.header.stamp = ros_clock.now();
    //size_t n = (size_t)(dis(gen) * 500 * 1000);
    message.ranges.resize(data_sizes[count_]);
    if (fcnt++ == 200) {
        fcnt = 0;
        count_++;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing scan: '%d'", message.ranges.size());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
