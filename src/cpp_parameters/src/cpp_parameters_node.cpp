/*
 * @Descripttion: 
 * @Author: jev
 * @version: 
 * @Date: 2025-02-05
 * @LastEditors: jev
 * @LastEditTime: 2025-02-05
 */
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    // 添加参数描述
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    // 创建一个参数 名为：my_parameter 默认参数为：world
    this->declare_parameter("my_parameter", "world", param_desc);

    // 每1s执行一次回调函数
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    // 从node中获取参数 my_parameter，将其保存到my_param
    std::string my_param = this->get_parameter("my_parameter").as_string();
 
    // 打印
    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    // 将参数设置为默认值
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}