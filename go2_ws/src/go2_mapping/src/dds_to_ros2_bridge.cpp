#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace unitree::robot;

class DdsToRos2Bridge : public rclcpp::Node
{
public:
    DdsToRos2Bridge(const std::string& network_interface)
        : Node("dds_to_ros2_bridge")
    {
        // Initialize the DDS channel
        ChannelFactory::Instance()->Init(0, network_interface);

        // Initialize the DDS subscriber
        subscriber_ = std::make_shared<ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>("rt/utlidar/cloud");
        subscriber_->InitChannel(std::bind(&DdsToRos2Bridge::ddsCallback, this, std::placeholders::_1));

        // Initialize the ROS2 publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/point_cloud", 10);
    }

private:
    void ddsCallback(const void* message)
    {
        const sensor_msgs::msg::dds_::PointCloud2_* dds_msg = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

        // Convert DDS message to ROS2 message
        sensor_msgs::msg::PointCloud2 ros2_msg;
        convertPointCloud2(dds_msg, ros2_msg);

        // Publish the ROS2 message
        publisher_->publish(ros2_msg);
    }

    void convertPointCloud2(const sensor_msgs::msg::dds_::PointCloud2_* dds_msg, sensor_msgs::msg::PointCloud2& ros2_msg)
    {
        // Header
        ros2_msg.header.stamp.sec = dds_msg->header().stamp().sec();
        ros2_msg.header.stamp.nanosec = dds_msg->header().stamp().nanosec();
        ros2_msg.header.frame_id = dds_msg->header().frame_id();

        // Metadata
        ros2_msg.height = dds_msg->height();
        ros2_msg.width = dds_msg->width();
        ros2_msg.is_bigendian = dds_msg->is_bigendian();
        ros2_msg.point_step = dds_msg->point_step();
        ros2_msg.row_step = dds_msg->row_step();
        ros2_msg.is_dense = dds_msg->is_dense();

        // Fields
        for (const auto& field_dds : dds_msg->fields())
        {
            sensor_msgs::msg::PointField field_ros2;
            field_ros2.name = field_dds.name();
            field_ros2.offset = field_dds.offset();
            field_ros2.datatype = field_dds.datatype();
            field_ros2.count = field_dds.count();
            ros2_msg.fields.push_back(field_ros2);
        }

        // Data
        size_t data_size = dds_msg->data().size();
        ros2_msg.data.resize(data_size);
        memcpy(ros2_msg.data.data(), dds_msg->data().data(), data_size);
    }

    std::shared_ptr<ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: dds_to_ros2_bridge <network_interface>" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DdsToRos2Bridge>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
