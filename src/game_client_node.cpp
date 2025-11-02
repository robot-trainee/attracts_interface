#include "attracts_interface/game_client_node.hpp"

#include <rabcl/controller/omni_drive.hpp>

GameClient::GameClient() : Node("game_client_node")
{
    max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
    max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
    max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
    max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);

    cmd_pub_ = this->create_publisher<attracts_msgs::msg::AttractsCommand>("cmd", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    game_data_input_sub_ = this->create_subscription<attracts_msgs::msg::GameDataInput>(
        "/game_data_input", 10, std::bind(&GameClient::GameDataInputCB, this, std::placeholders::_1));
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(25ms, std::bind(&GameClient::TimerCB, this));
}

void GameClient::GameDataInputCB(const attracts_msgs::msg::GameDataInput::SharedPtr msg)
{
    game_data_input_msg_ = *msg;
}

void GameClient::TimerCB()
{
    attracts_msgs::msg::AttractsCommand cmd; // rad/s
    UpdateCmdVel(cmd);
    UpdatePositions(cmd);
}

void GameClient::UpdateCmdVel(attracts_msgs::msg::AttractsCommand& cmd)
{
    // --- 足回り
    // 並進
    if (game_data_input_msg_.key_w) {
        cmd.chassis_vel.x = max_omni_vel_;
    }
    if (game_data_input_msg_.key_s) {
        cmd.chassis_vel.x = -max_omni_vel_;
    }
    if (game_data_input_msg_.key_a) {
        cmd.chassis_vel.y = max_omni_vel_;
    }
    if (game_data_input_msg_.key_d) {
        cmd.chassis_vel.y = -max_omni_vel_;
    }
    // 回転
    cmd.chassis_vel.z = 0.0;

    // --- 砲塔
    // yaw
    cmd.yaw_pos =
        positions_.at(4) + (double)game_data_input_msg_.mouse_delta_x / 100.0;
    cmd.yaw_pos = std::fmod(cmd.yaw_pos, 2.0 * M_PI);
    if (cmd.yaw_pos < 0) {
        cmd.yaw_pos += 2.0 * M_PI;
    }
    // pitch
    cmd.pitch_pos =
        positions_.at(5) + (double)game_data_input_msg_.mouse_delta_y / 100.0;
    if (cmd.pitch_pos < -M_PI / 12)
    {
        cmd.pitch_pos = -M_PI / 12;
    }
    if (cmd.pitch_pos > M_PI / 6)
    {
        cmd.pitch_pos = M_PI / 6;
    }
    // 前回指令値を保存
    positions_.at(4) = cmd.yaw_pos;
    positions_.at(5) = cmd.pitch_pos;

    // --- 動作モード
    cmd.fire_mode = 0;
    cmd.load_mode = 0;
    cmd.speed_mode = 0;
    cmd.chassis_mode = 0;

    cmd_pub_->publish(cmd);
}

void GameClient::UpdatePositions(const attracts_msgs::msg::AttractsCommand& cmd)
{
    rabcl::OmniDrive omni_drive(wheel_d_, body_d_);
    std::array<double, 6> joint_vel;
    omni_drive.CalcVel(
        cmd.chassis_vel.x, cmd.chassis_vel.y, cmd.chassis_vel.z,
        joint_vel.at(0), joint_vel.at(1), joint_vel.at(2), joint_vel.at(3),
        positions_.at(4));
    positions_.at(4) = cmd.yaw_pos;
    positions_.at(5) = cmd.pitch_pos;

    double game_client_freq = 20.0; // Hz
    for (int i = 0; i < 6; i++)
    {
        positions_.at(i) += joint_vel.at(i) / game_client_freq;
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
    rclcpp::spin(std::make_shared<GameClient>());
    rclcpp::shutdown();
    return 0;
}
