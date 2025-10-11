#include "go2_remap/go2_remap.hpp"

Go2Remap::Go2Remap() : Node("go2_remap")
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/utlidar/robot_pose", rclcpp::QoS(10),
        std::bind(&Go2Remap::pose_callback, this, std::placeholders::_1));

    joint_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&Go2Remap::joint_callback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
}

// **Pose Callback (Updates `world -> odom` dynamically)**
void Go2Remap::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(tf_mutex_);

    rclcpp::Time timestamp = msg->header.stamp;
    if (timestamp.nanoseconds() == 0)
    {
        timestamp = this->now();
    }

    // **Continuously Update `world -> odom` (DYNAMIC)**
    world_odom_tf_.header.stamp = timestamp;
    world_odom_tf_.header.frame_id = "world";
    world_odom_tf_.child_frame_id = "odom";
    world_odom_tf_.transform.translation.x = msg->pose.position.x;
    world_odom_tf_.transform.translation.y = msg->pose.position.y;
    world_odom_tf_.transform.translation.z = msg->pose.position.z;
    world_odom_tf_.transform.rotation = msg->pose.orientation;
}

// **Joint Callback (Static `odom -> base_link` + Joints)**
void Go2Remap::joint_callback(const unitree_go::msg::LowState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(tf_mutex_);

    rclcpp::Time timestamp = this->get_clock()->now();
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = timestamp;

    std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
    std::vector<double> joint_positions;

    // **Publish stored `world -> odom` transform**
    // **Ensure `world -> odom` is published**
    // if (!world_odom_tf_.header.frame_id.empty() && !world_odom_tf_.child_frame_id.empty())
    // {
    //     world_odom_tf_.header.stamp = timestamp;  // Update timestamp
    //     tf_transforms.push_back(world_odom_tf_);
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Skipping `world -> odom` transform: Frame IDs are missing!");
    // }

    // **Static `odom -> base_link` (Fixed Transform)**
    geometry_msgs::msg::TransformStamped odom_base_tf;
    odom_base_tf.header.stamp = timestamp;
    odom_base_tf.header.frame_id = "odom";
    odom_base_tf.child_frame_id = "base_link";
    odom_base_tf.transform.translation.x = 0.0;
    odom_base_tf.transform.translation.y = 0.0;
    odom_base_tf.transform.translation.z = 0.0;
    odom_base_tf.transform.rotation.w = 1.0;
    tf_transforms.push_back(odom_base_tf);

    // **Loop through joints & Ignore collision-only links**
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        joint_positions.push_back(msg->motor_state[i].q);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = timestamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = joint_names_[i];
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_transforms.push_back(tf_msg);
    }

    // **Publish All Transforms & Joint States**
    tf_broadcaster_->sendTransform(tf_transforms);
    joint_state_msg.name = joint_names_;
    joint_state_msg.position = joint_positions;
    joint_state_msg.velocity = std::vector<double>(joint_names_.size(), 0.0);
    joint_state_msg.effort = std::vector<double>(joint_names_.size(), 0.0);
    joint_state_pub_->publish(joint_state_msg);
}

// **Main**
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2Remap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
