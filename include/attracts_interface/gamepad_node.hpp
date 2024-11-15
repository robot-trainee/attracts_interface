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
    void TimerCB();
    void UpdateCmdVel(std_msgs::msg::Float32MultiArray& cmd_vel);
    void UpdatePositions(const std_msgs::msg::Float32MultiArray& cmd_vel);

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    double wheel_d_ = 0.06; // m
    double body_d_ = 0.28; // m
    double max_omni_vel_ = 0.35; // m/s
    double max_omni_rot_vel_ = 60.0; // deg/s
    double max_yaw_rot_vel_ = 120.0; // deg/s ギア比38/25
    double max_pitch_rot_vel_ = 120.0; // deg/s ギア比34/29
    sensor_msgs::msg::Joy joy_msg_;
    std::array<double, 6> positions_;
};

#endif
