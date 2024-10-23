#include "attracts_interface/gamepad_node.hpp"

Gamepad::Gamepad() : Node("gamepad_node")
{
    // TODO: launchでparamがロードできない、なんで？
    // max_trans_vel_ = this->get_parameter("max_trans_vel").as_double();
    // RCLCPP_INFO(this->get_logger(), "max_trans_vel: %lf", max_trans_vel_);
    // max_rot_vel_ = this->get_parameter("max_rot_vel").as_double();
    // RCLCPP_INFO(this->get_logger(), "max_rot_vel: %lf", max_rot_vel_);
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("cmd_vel", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Gamepad::JoyCB, this, std::placeholders::_1));
}

void Gamepad::JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    double vec_x = 1.0 * msg->axes.at(1);
    double vec_y = -1.0 * msg->axes.at(0);
    double theta = std::atan2(vec_y, vec_x);
    double norm = std::sqrt(std::pow(vec_x, 2) + std::pow(vec_y, 2));

    double max_omni_vel = 0.1;
    double max_omni_rot_vel = 0.05;
    double max_yaw_vel = 0.05;
    double max_pitch_vel = 0.02;
    std_msgs::msg::Float32MultiArray cmd_vel;
    cmd_vel.data.resize(6);
    cmd_vel.data.at(0) = max_omni_vel * norm * std::sin(theta - M_PI * 0.25);
    cmd_vel.data.at(1) = max_omni_vel * norm * std::sin(theta + M_PI * 0.25);
    cmd_vel.data.at(2) = max_omni_vel * norm * std::sin(theta + M_PI * 1.25);
    cmd_vel.data.at(3) = max_omni_vel * norm * std::sin(theta - M_PI * 1.25);
    for (int i = 0; i < 4; i++)
    {
        if (msg->buttons.at(4))
        {
            cmd_vel.data.at(i) += max_omni_rot_vel;
        }
        if (msg->buttons.at(5))
        {
            cmd_vel.data.at(i) -= max_omni_rot_vel;
        }
    }
    cmd_vel.data.at(4) = -max_yaw_vel * msg->axes.at(3);
    cmd_vel.data.at(5) = max_pitch_vel * msg->axes.at(4);
    cmd_pub_->publish(cmd_vel);

    for (int i = 0; i < 6; i++)
    {
        positions_.at(i) += cmd_vel.data.at(i);
    }

    if (positions_.at(5) < -M_PI / 12)
    {
        positions_.at(5) = -M_PI / 12;
    }
    if (positions_.at(5) > M_PI / 6)
    {
        positions_.at(5) = M_PI / 6;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_clock()->now();
    joint_state.name.emplace_back("right_front_wheel_joint");
    joint_state.name.emplace_back("left_front_wheel_joint");
    joint_state.name.emplace_back("right_back_wheel_joint");
    joint_state.name.emplace_back("left_back_wheel_joint");
    joint_state.name.emplace_back("yaw_joint");
    joint_state.name.emplace_back("pitch_joint");
    joint_state.position.emplace_back(positions_.at(0));
    joint_state.position.emplace_back(positions_.at(1));
    joint_state.position.emplace_back(positions_.at(2));
    joint_state.position.emplace_back(positions_.at(3));
    joint_state.position.emplace_back(positions_.at(4));
    joint_state.position.emplace_back(positions_.at(5));
    joint_state_pub_->publish(joint_state);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gamepad>());
    rclcpp::shutdown();
    return 0;
}
