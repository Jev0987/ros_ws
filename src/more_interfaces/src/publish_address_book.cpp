/*
 * @Descripttion: 
 * @Author: jev
 * @version: 
 * @Date: 2025-02-05
 * @LastEditors: jev
 * @LastEditTime: 2025-02-05
 */
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// 包含了要发布数据的消息类型
#include "more_interfaces/msg/address_book.hpp"


using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    // 创建发布者，消息类型为 AddressBook，topic 名为 address_book
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);  // 10 is the queue size
    
    // 定义一个lambda函数，每秒发布一次消息
    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.phone_number = "1234567890";
        message.phone_type = message.PHONE_TYPE_MOBILE;

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    // 创建定时器，每秒发布一次消息
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}