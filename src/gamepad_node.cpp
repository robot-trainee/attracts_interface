#include "attracts_interface/gamepad_node.hpp"

#include <rabcl/controller/omni_drive.hpp>

Gamepad::Gamepad() : Node("gamepad_node")
{
    max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
    max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
    max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
    max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);

    cmd_pub_ = this->create_publisher<attracts_msgs::msg::AttractsCommand>("cmd", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Gamepad::JoyCB, this, std::placeholders::_1));
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(25ms, std::bind(&Gamepad::TimerCB, this));
}

void Gamepad::JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_msg_ = *msg;
}

void Gamepad::TimerCB()
{
    attracts_msgs::msg::AttractsCommand cmd; // rad/s
    if (joy_msg_.axes.size() > 0 && joy_msg_.buttons.size() > 0)
    {
        UpdateCmdVel(cmd);
        UpdatePositions(cmd);
    }
}

void Gamepad::UpdateCmdVel(attracts_msgs::msg::AttractsCommand& cmd)
{
    cmd.chassis_vel_x.data = max_omni_vel_ * joy_msg_.axes.at(1);
    cmd.chassis_vel_y.data = max_omni_vel_ * joy_msg_.axes.at(0);

    if (joy_msg_.buttons.at(6) == 1)
    {
        cmd.chassis_vel_z.data = max_omni_rot_vel_;
    }
    else if (joy_msg_.buttons.at(7) == 1)
    {
        cmd.chassis_vel_z.data = -1,0 * max_omni_rot_vel_;
    }
    else
    {
        cmd.chassis_vel_z.data = 0.0;
    }

    cmd.yaw_vel.data = max_yaw_rot_vel_ * joy_msg_.axes.at(3);
    cmd.pitch_vel.data = max_pitch_rot_vel_ * joy_msg_.axes.at(4);

    cmd.fire_mode.data = joy_msg_.buttons.at(5);
    cmd.load_mode.data = joy_msg_.buttons.at(4);
    if (joy_msg_.buttons.at(0) == 1)
    {
        cmd.load_mode.data = 2;
    }
    cmd.speed_mode.data = 0;
    cmd.chassis_mode.data = 0;
    cmd_pub_->publish(cmd);
}

void Gamepad::UpdatePositions(const attracts_msgs::msg::AttractsCommand& cmd_vel)
{
    rabcl::OmniDrive omni_drive(wheel_d_, body_d_);
    std::array<double, 6> joint_vel;
    omni_drive.CalcVel(
        cmd_vel.chassis_vel_x.data, cmd_vel.chassis_vel_y.data, cmd_vel.chassis_vel_z.data,
        joint_vel.at(0), joint_vel.at(1), joint_vel.at(2), joint_vel.at(3));
    joint_vel.at(4) = cmd_vel.yaw_vel.data;
    joint_vel.at(5) = cmd_vel.pitch_vel.data;

    double joy_freq = 40.0; // Hz
    for (int i = 0; i < 6; i++)
    {
        positions_.at(i) += joint_vel.at(i) / joy_freq;
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
