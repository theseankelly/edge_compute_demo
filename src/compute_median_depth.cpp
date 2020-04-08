#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ifm3d_ros2/camera_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

/*
 * Helper node which subscribes point clouds and publishes the median depth
 */
class ComputeMedianDepth: public rclcpp::Node
{
public:
  ComputeMedianDepth()
  : Node("compute_median_depth")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/cloud",
      rclcpp::SensorDataQoS(),
      std::bind(
        &ComputeMedianDepth::cloud_callback,
        this,
        std::placeholders::_1));
  }
private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // TODO: Actually compute and publish the median depth
    RCLCPP_INFO(this->get_logger(), "Got a pointcloud (%d, %d)!",
      msg->height, msg->width);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto camera = std::make_shared<ifm3d_ros2::CameraNode>(options);
  exec.add_node(camera->get_node_base_interface());
  auto compute_median_depth = std::make_shared<ComputeMedianDepth>();
  exec.add_node(compute_median_depth);

  // Activate the camera (a managed node)
  //
  // This is a bit of a hack and should be done through a launch script,
  // but there's no python (and thus no ros launch) available embedded on the
  // O3D303.
  camera->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  camera->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}

