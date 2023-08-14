#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudSubscriberNode : public rclcpp::Node
{
public:
  PointCloudSubscriberNode() : Node("pointcloud_subscriber_node")
  {
    // Get the list of point cloud topics from parameter server
    this->declare_parameter("pointcloud_topics", std::vector<std::string>());
    this->get_parameter("pointcloud_topics", pointcloud_topics_);

    // Create subscriptions for each point cloud topic
    for (const std::string& topic : pointcloud_topics_) {
      subscriptions_.emplace_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, 10, std::bind(&PointCloudSubscriberNode::cloudCallback, this, std::placeholders::_1))
      );
    }
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // This callback will be called for each subscribed topic.
    // Since you mentioned not doing anything, the implementation is left empty.
  }

  std::vector<std::string> pointcloud_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
