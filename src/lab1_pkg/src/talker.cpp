#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto talker_node = std::make_shared<rclcpp::Node>("talker");

    talker_node->declare_parameter("v", 0.0);
    talker_node->declare_parameter("d", 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    auto pub = talker_node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", qos);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(talker_node);

    while (rclcpp::ok()) {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.drive.speed = static_cast<float>(talker_node->get_parameter("v").as_double());
        msg.drive.steering_angle = static_cast<float>(talker_node->get_parameter("d").as_double());
        pub->publish(msg);
        executor.spin_some();
    }
    rclcpp::shutdown();
    return 0;
}