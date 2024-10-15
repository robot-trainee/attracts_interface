#include "attracts_interface/gamepad_node.hpp"

Gamepad::Gamepad() : Node("gamepad_node")
{
    // TODO: launchでparamがロードできない、なんで？
    // max_trans_vel_ = this->get_parameter("max_trans_vel").as_double();
    // RCLCPP_INFO(this->get_logger(), "max_trans_vel: %lf", max_trans_vel_);
    // max_rot_vel_ = this->get_parameter("max_rot_vel").as_double();
    // RCLCPP_INFO(this->get_logger(), "max_rot_vel: %lf", max_rot_vel_);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Gamepad::JoyCB, this, std::placeholders::_1));
}

void Gamepad::JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg) const
{
    geometry_msgs::msg::Twist cmd_vel;
    // cmd_vel.linear.x = max_trans_vel_ * msg->axes.at(1);
    // cmd_vel.linear.y = -1.0 * max_trans_vel_ * msg->axes.at(0);
    // cmd_vel.angular.z = -1.0 * max_rot_vel_ * msg->axes.at(3);
    cmd_vel.linear.x = 0.2 * msg->axes.at(1);
    cmd_vel.linear.y = -1.0 * 0.2 * msg->axes.at(0);
    cmd_vel.angular.z = -1.0 * 0.4 * msg->axes.at(3);
    cmd_pub_->publish(cmd_vel);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gamepad>());
    rclcpp::shutdown();
    return 0;
}
