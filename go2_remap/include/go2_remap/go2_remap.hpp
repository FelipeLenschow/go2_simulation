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
        "1_FR_hip_joint", "1_FR_thigh_joint", "1_FR_calf_joint",
        "2_FL_hip_joint", "2_FL_thigh_joint", "2_FL_calf_joint",
        "3_RR_hip_joint", "3_RR_thigh_joint", "3_RR_calf_joint",
        "4_RL_hip_joint", "4_RL_thigh_joint", "4_RL_calf_joint"
    };        

    const std::vector<std::string> collision_links_ = {
        "1_FR_calflower", "1_FR_calflower1",
        "2_FL_calflower", "2_FL_calflower1",   
        "3_RR_calflower", "3_RR_calflower1",     
        "4_RL_calflower", "4_RL_calflower1"
    };        
};

#endif // GO2_REMAP_HPP
