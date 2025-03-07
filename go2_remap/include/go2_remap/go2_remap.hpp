#ifndef GO2_REMAP_HPP
#define GO2_REMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <mutex>

class Go2Remap : public rclcpp::Node
{
public:
    Go2Remap();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void joint_callback(const unitree_go::msg::LowState::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr joint_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex tf_mutex_;
    geometry_msgs::msg::TransformStamped world_odom_tf_;
    bool world_odom_published_ = false;

    const std::vector<std::string> joint_names_ = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    const std::vector<std::string> collision_links_ = {
        "FR_calflower", "FR_calflower1",
        "FL_calflower", "FL_calflower1",
        "RR_calflower", "RR_calflower1",
        "RL_calflower", "RL_calflower1"};
};

#endif // GO2_REMAP_HPP
