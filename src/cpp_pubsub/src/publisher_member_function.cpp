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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 包含了常用的ROS2模块
#include "rclcpp/rclcpp.hpp"

// 包含了要发布数据的消息类型
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
// 发布者类继承自 Node 类型
class MinimalPublisher : public rclcpp::Node
{
public:
  // 构造函数将node命名为：minimal_publisher，将count_设置为0
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // publisher 初始化消息类型为 Num、topic 命名为 topic
    // 发生备份时用到的限制消息数量的队列大小为 10
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
    // 初始化定时器，这将会使timer_callback函数被每秒执行两次
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // 具体的数据发布函数
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();
    message.num = this->count_++;

    // RCLCPP_INFO_STREAM 用于打印信息
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  // 处理来自node的数据，包括来自定时器的回调。
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
