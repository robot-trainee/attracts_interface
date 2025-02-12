#ifndef GAMEPAD_NODE_HPP
#define GAMEPAD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <attracts_msgs/msg/attracts_command.hpp>

class Gamepad : public rclcpp::Node
{
public:
    Gamepad();

private:
    void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg);
    void TimerCB();
    void UpdateCmdVel(attracts_msgs::msg::AttractsCommand& cmd);
    void UpdatePositions(const attracts_msgs::msg::AttractsCommand& cmd);

private:
    rclcpp::Publisher<attracts_msgs::msg::AttractsCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    double wheel_d_ = 0.06; // m
    double body_d_ = 0.28; // m
    double max_omni_vel_; // m/s
    double max_omni_rot_vel_; // rad/s
    double max_yaw_rot_vel_; // rad/s
    double max_pitch_rot_vel_; // rad/s

    sensor_msgs::msg::Joy joy_msg_;
    std::array<double, 6> positions_;
};

#endif
