/*
 * @Descripttion: 
 * @Author: jev
 * @version: 
 * @Date: 2025-02-05
 * @LastEditors: jev
 * @LastEditTime: 2025-02-05
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
// 包含了要发布数据的消息类型
#include "more_interfaces/msg/address_book.hpp"

class AddressBookSubscriber : public rclcpp::Node {
public:
    AddressBookSubscriber() : Node("address_book_subscriber") {
        // 使用 create_subscription 创建订阅者
        auto subscribe_msg = [this](const more_interfaces::msg::AddressBook &msg) -> void {
            std::cout << "Subscribing message : \n" <<
            "User: " << msg.first_name <<" " << msg.last_name << "\n" << 
            "Phone number: " << msg.phone_number << "\n" << 
            "Phone type: " << std::to_string(msg.phone_type) << std::endl;
        };
        subscription_ = this->create_subscription<more_interfaces::msg::AddressBook>(
            "address_book", 10, subscribe_msg
        );
    }
private:
    rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookSubscriber>());
    rclcpp::shutdown();

    return 0;
}