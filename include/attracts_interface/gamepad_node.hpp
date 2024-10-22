#ifndef GAMEPAD_NODE_HPP
#define GAMEPAD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class Gamepad : public rclcpp::Node
{
public:
    Gamepad();

private:
    void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg);

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

private:
    double max_trans_vel_;
    double max_rot_vel_;
    std::array<double, 6> positions_;
};

#endif
