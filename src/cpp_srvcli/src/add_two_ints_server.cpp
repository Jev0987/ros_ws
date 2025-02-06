/*
 * @Descripttion: 
 * @Author: jev
 * @version: 
 * @Date: 2025-02-05
 * @LastEditors: jev
 * @LastEditTime: 2025-02-05
 */
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

// add 函数 用于把两个request的a和b相加，然后把结果赋值给response的sum
// 通过RCLCPP_INFO打印出来
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建节点，名为add_two_ints_server
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
  
  // 创建服务，名为add_two_ints，回调函数为add
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    
  // 打印信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  
  // 进入spin循环
  rclcpp::spin(node);
  rclcpp::shutdown();
}