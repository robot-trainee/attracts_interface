#ifndef GAMEPAD_NODE_HPP
#define GAMEPAD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Gamepad : public rclcpp::Node
{
public:
    Gamepad();

private:
    void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg) const;

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

private:
    double max_trans_vel_;
    double max_rot_vel_;
};

#endif
