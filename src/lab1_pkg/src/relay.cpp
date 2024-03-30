#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    auto relay_node = std::make_shared<rclcpp::Node>("relay");
    
    auto pub = relay_node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_relay", qos);
    
    auto callback = [&pub](ackermann_msgs::msg::AckermannDriveStamped const &msg) {
        auto new_msg = msg;
        new_msg.drive.speed *= 3;
        new_msg.drive.steering_angle *= 3;
        pub->publish(new_msg);
    };

    auto sub = relay_node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive", qos, callback);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(relay_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}