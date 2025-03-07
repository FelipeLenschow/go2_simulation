#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        // ✅ Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ✅ Create Subscriber for PointCloud2
        pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, "/utlidar/cloud");

        // ✅ Create Publisher for Transformed PointCloud2
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_cloud", 10);

        // ✅ Register Callback
        pointcloud_sub_->registerCallback(
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        // ✅ Lookup Transform: "world" -> "utlidar_lidar"
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(
                "world", msg->header.frame_id, tf2::TimePointZero);

            // ✅ Transform PointCloud
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);

            // ✅ Publish Transformed PointCloud
            pointcloud_pub_->publish(transformed_cloud);
            // RCLCPP_INFO(this->get_logger(), "Published transformed point cloud");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}